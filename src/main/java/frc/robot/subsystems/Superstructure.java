package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Constants;

public class Superstructure extends SubsystemBase {

    // State machine with encoder presets
    public static enum SuperstructurePreset {
        STOW(0, 0, 0, 0, "Stow"),
        RECEIVE(0, 0, 0, 0, "Receive"),
        L1(0, 0, 0, 0, "L1"),
        L2(0, 0, 0, 0, "L2"),
        L3(0, 0, 0, 0, "L3"),
        L4(0, 0, 0, 0, "L4"),
        NONE(0, 0, 0, 0, "None"); // being manually overridden to something
        public double elevatorRotations;
        public double pivotRadians;
        public double leftBeltVoltage;
        public double rightBeltVoltage;
        public String name;
        SuperstructurePreset(double elevatorRotations, double pivotRadians, double leftBeltVoltage, double rightBeltVoltage, String name) {
            this.elevatorRotations = elevatorRotations;
            this.pivotRadians = pivotRadians;
            this.leftBeltVoltage = leftBeltVoltage;
            this.rightBeltVoltage = rightBeltVoltage;
            this.name = name;
        }
    }

    private SuperstructurePreset state = SuperstructurePreset.STOW;

    // Elevator
    private final TalonFX
        elevatorA = new TalonFX(Constants.Elevator.ELEVATOR_ID_A, Constants.RIO_BUS_NAME),
        elevatorB = new TalonFX(Constants.Elevator.ELEVATOR_ID_B, Constants.RIO_BUS_NAME);
    private final MotionMagicVoltage mmVoltageReq = new MotionMagicVoltage(0);
    private final Follower followReq = new Follower(Constants.Elevator.ELEVATOR_ID_A, false);
    // Manipulator
    private final SparkMax
        beltLeft = new SparkMax(Constants.Manipulator.BELT_LEFT_ID, MotorType.kBrushless),
        beltRight = new SparkMax(Constants.Manipulator.BELT_RIGHT_ID, MotorType.kBrushless);
    private final SparkFlex pivot = new SparkFlex(Constants.Manipulator.PIVOT_ID, MotorType.kBrushless);
    private final CANcoder pivotCancoder = new CANcoder(Constants.Manipulator.CANCODER_ID);

    public Superstructure() {
        // Elevator
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic
            .withMotionMagicCruiseVelocity(Constants.Elevator.ELEVATOR_MM_VEL)
            .withMotionMagicAcceleration(Constants.Elevator.ELEVATOR_MM_ACCEL);
        config.Slot0
            .withKP(Constants.Elevator.P)
            .withKI(Constants.Elevator.I)
            .withKD(Constants.Elevator.D)
            .withKS(Constants.Elevator.kS)
            .withKG(Constants.Elevator.kG)
            .withKV(Constants.Elevator.kV)
            .withKA(Constants.Elevator.kS);
        elevatorA.getConfigurator().apply(config);
        elevatorB.getConfigurator().apply(config);
        // Manipulator
        SparkFlexConfig pivotConfig = new SparkFlexConfig();
        // Both the pivot and its cancoder will operate in radians
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI * Constants.Manipulator.PIVOT_RATIO); // Convert to radians
        recalibratePivot();
        pivotConfig.closedLoop.p(Constants.Manipulator.P).i(Constants.Manipulator.I).d(Constants.Manipulator.D);
        pivotConfig.softLimit.forwardSoftLimit(Constants.Manipulator.MAX_RADIANS);
        pivotConfig.softLimit.reverseSoftLimit(Constants.Manipulator.MIN_RADIANS);
        pivotConfig.softLimit.reverseSoftLimitEnabled(true);
        pivotConfig.softLimit.forwardSoftLimitEnabled(true);
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setPreset(SuperstructurePreset.STOW);
    }

    // Set elevator position
    private void setElevator(double rotations) {
        // TODO - Not sure if this is the correct way to do encoder limits with TalonFX controllers
        elevatorA.setControl(mmVoltageReq.withPosition(rotations)
            .withLimitForwardMotion(elevatorA.getPosition().getValueAsDouble() > Constants.Elevator.MAX_ROTATIONS)
            .withLimitReverseMotion(elevatorA.getPosition().getValueAsDouble() < 0)
        );
        elevatorB.setControl(followReq);
    }

    private void recalibratePivot() {
      pivot.getEncoder().setPosition(2 * Math.PI * pivotCancoder.getAbsolutePosition().getValueAsDouble());
    }

    public Command setPreset(SuperstructurePreset preset) {
        return this.run(() -> {
            setElevator(preset.elevatorRotations);
            pivot.getClosedLoopController().setReference(preset.pivotRadians, ControlType.kPosition);
            beltLeft.setVoltage(preset.leftBeltVoltage);
            beltRight.setVoltage(preset.rightBeltVoltage);
            state = preset;
        });
    }

    public Command setManual(double elevatorRotations, double pivotRadians, double leftBeltVoltage, double rightBeltVoltage) {
        return this.run(() -> {
            setElevator(elevatorRotations);
            pivot.getClosedLoopController().setReference(pivotRadians, ControlType.kPosition);
            beltLeft.setVoltage(leftBeltVoltage);
            beltRight.setVoltage(rightBeltVoltage);
            state = SuperstructurePreset.NONE;
        });
    }

    public SuperstructurePreset getState() {
        return state;
    }
}
