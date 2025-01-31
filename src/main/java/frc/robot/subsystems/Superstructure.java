package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utilities.lists.Constants;

public class Superstructure extends SubsystemBase {

    // State machine with encoder presets
    public static enum SuperstructurePreset {
        STOW(0, 0, 0, 0, "Stow"),
        RECEIVE(0, 0, 0, 0, "Receive"),
        L1(0, 0, 0, 0, "1"),
        L2(0, 0, 0, 0, "2"),
        L3(0, 0, 0, 0, "3"),
        L4(0, 0, 0, 0, "4"),
        L1_GO(0, 0, 0.2, 0.2, "1"),
        L2_GO(0, 0, 0.2, 0.2, "2"),
        L3_GO(0, 0, 0.2, 0.2, "3"),
        L4_GO(0, 0, 0.2, 0.2, "4"),
        NONE(0, 0, 0, 0, "None"); // being manually overridden to something
        public double elevatorRotations;
        public double pivotRadians;
        public double leftBelt;
        public double rightBelt;
        public String name;
        SuperstructurePreset(double elevatorRotations, double pivotRadians, double leftBelt, double rightBelt, String name) {
            this.elevatorRotations = elevatorRotations;
            this.pivotRadians = pivotRadians;
            this.leftBelt = leftBelt;
            this.rightBelt = rightBelt;
            this.name = name;
        }

        public static SuperstructurePreset getCorrespondingGoState(SuperstructurePreset preset) {
            switch (preset) {
                case L1: return L1_GO;
                case L2: return L2_GO;
                case L3: return L3_GO;
                case L4: return L4_GO;
                default: return NONE;
            }
        }
    }

    private SuperstructurePreset state = SuperstructurePreset.STOW;

    // Elevator
    private final TalonFX
        elevatorA = new TalonFX(Constants.Elevator.ELEVATOR_ID_A, Constants.RIO_BUS_NAME),
        elevatorB = new TalonFX(Constants.Elevator.ELEVATOR_ID_B, Constants.RIO_BUS_NAME);
    private final MotionMagicVoltage mmVoltageReq = new MotionMagicVoltage(0).withSlot(0);
    private final Follower followReq = new Follower(Constants.Elevator.ELEVATOR_ID_A, false);
    // Manipulator
    private final SparkMax
        beltLeft = new SparkMax(Constants.Manipulator.BELT_LEFT_ID, MotorType.kBrushless),
        beltRight = new SparkMax(Constants.Manipulator.BELT_RIGHT_ID, MotorType.kBrushless);
    private final SparkFlex pivot = new SparkFlex(Constants.Manipulator.PIVOT_ID, MotorType.kBrushless);
    private final CANcoder pivotCancoder = new CANcoder(Constants.Manipulator.CANCODER_ID);
    private double pivotEncoderTarget = 0; // This is a little finicky but I can't figure out how to directly read closed loop error from the Spark Flex.

    public Superstructure() {
        // Elevator
        // For elevator control see https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
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
        // For pivot control see: https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
        SparkFlexConfig pivotConfig = new SparkFlexConfig();
        // Both the pivot and its cancoder will operate in radians
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI * Constants.Manipulator.PIVOT_RATIO); // Convert to radians
        recalibratePivot();
        pivotConfig.closedLoop.pid(Constants.Manipulator.P, Constants.Manipulator.I, Constants.Manipulator.D);
        pivotConfig.closedLoop.maxMotion
            .maxVelocity(Constants.Manipulator.MAX_VELOCITY)
            .maxAcceleration(Constants.Manipulator.MAX_ACCEL)
            .allowedClosedLoopError(Constants.Manipulator.ROTATION_TOLERANCE);
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
            pivot.getClosedLoopController().setReference(preset.pivotRadians, ControlType.kMAXMotionPositionControl);
            beltLeft.set(preset.leftBelt);
            beltRight.set(preset.rightBelt);
            state = preset;
            pivotEncoderTarget = preset.pivotRadians;
        });
    }

    public Command setManual(DoubleSupplier elevatorRotations, DoubleSupplier pivotRadians, DoubleSupplier leftBelt, DoubleSupplier rightBelt) {
        return this.run(() -> {
            setElevator(elevatorRotations.getAsDouble());
            pivot.getClosedLoopController().setReference(pivotRadians.getAsDouble(), ControlType.kMAXMotionPositionControl);
            beltLeft.set(leftBelt.getAsDouble());
            beltRight.set(rightBelt.getAsDouble());
            state = SuperstructurePreset.NONE;
            pivotEncoderTarget = pivotRadians.getAsDouble();
        });
    }

    public SuperstructurePreset getState() {
        return state;
    }

    public boolean atSetpoint() {
        return elevatorA.getClosedLoopError().getValueAsDouble() < Constants.Elevator.ROTATION_TOLERANCE
            && Math.abs(pivot.getEncoder().getPosition() - pivotEncoderTarget) < Constants.Manipulator.ROTATION_TOLERANCE;
    }

    // SysID to find elevator gains
    // Should be safe and unlikely to damage anything IF the soft limits are working as intended.
    private final VoltageOut voltReq = new VoltageOut(0.0);

    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
        null,        // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    elevatorA.setControl(voltReq.withOutput(volts.in(Volts))
                        .withLimitForwardMotion(elevatorA.getPosition().getValueAsDouble() > Constants.Elevator.MAX_ROTATIONS)
                        .withLimitReverseMotion(elevatorA.getPosition().getValueAsDouble() < 0)
                    );
                    elevatorB.setControl(followReq);
                },
            null,
                this
            )
        );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
       return sysIdRoutine.dynamic(direction);
    }
}
