package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.ButtonBox.Button;
import frc.robot.utilities.lists.Constants;

public class Superstructure extends SubsystemBase {

    // State machine with encoder presets
    public static enum SuperstructurePreset {
        STOW_LOWER(0.1, Constants.Manipulator.MIN_ROTATIONS, 0, 0, "Stow Lower", Button.STOW_LOWER_PRESET),
        STOW_UPPER(0.1, Constants.Manipulator.MAX_ROTATIONS, 0, 0, "Stow Upper", Button.STOW_UPPER_PRESET),
        RECEIVE(1.9, -0.111816, 0, 0, "Receive", Button.RECEIVE_PRESET),
        L1(0.1, Constants.Manipulator.CLEAR_OF_ELEVATOR_ROTATIONS, 0, 0, "1", Button.L1_PRESET),
        L2(0.1, 0.094482, 0, 0, "2", Button.L2_PRESET),
        L3(4.193848, 0.118408, 0, 0, "3", Button.L3_PRESET),
        L4(14.85209, 0.077148, 0, 0, "4", Button.L4_PRESET),
        L1_GO(L1.elevatorRotations, L1.pivotRotations, 1, 1, "1", null),
        L2_GO(L2.elevatorRotations, L2.pivotRotations, 1, 1, "2", null),
        L3_GO(L3.elevatorRotations, L3.pivotRotations, 1, 1, "3", null),
        L4_GO(L4.elevatorRotations, L4.pivotRotations, 1, 1, "4", null),
        NONE(0, 0, 0, 0, "None", null); // being manually overridden to something
        public double elevatorRotations;
        public double pivotRotations;
        public double leftBelt;
        public double rightBelt;
        public String name;
        public Button button;
        SuperstructurePreset(double elevatorRotations, double pivotRotations, double leftBelt, double rightBelt, String name, Button button) {
            this.elevatorRotations = elevatorRotations;
            this.pivotRotations = pivotRotations;
            this.leftBelt = leftBelt;
            this.rightBelt = rightBelt;
            this.name = name;
            this.button = button;
        }

        public static SuperstructurePreset getCorrespondingGoState(SuperstructurePreset preset) {
            switch (preset) {
                case L1: return L1_GO;
                case L2: return L2_GO;
                case L3: return L3_GO;
                case L4: return L4_GO;
                default: return preset;
            }
        }
    }

    private SuperstructurePreset state = SuperstructurePreset.STOW_LOWER;

    // Elevator
    @Logged(name = "Elevator")
    private final TalonFX
        elevatorA = new TalonFX(Constants.Elevator.ELEVATOR_ID_A);
    private final TalonFX
        elevatorB = new TalonFX(Constants.Elevator.ELEVATOR_ID_B);
    StringLogEntry testState = new StringLogEntry(DataLogManager.getLog(), "testState");
    private final MotionMagicVoltage mmVoltageReq = new MotionMagicVoltage(0).withSlot(0);
    private final Follower followReq = new Follower(Constants.Elevator.ELEVATOR_ID_A, false);
    // Manipulator
    private final SparkMax
        beltLeft = new SparkMax(Constants.Manipulator.BELT_LEFT_ID, MotorType.kBrushless),
        beltRight = new SparkMax(Constants.Manipulator.BELT_RIGHT_ID, MotorType.kBrushless);
    private final TalonFX pivot = new TalonFX(Constants.Manipulator.PIVOT_ID);
    private final MotionMagicVoltage pivotReq = new MotionMagicVoltage(0).withSlot(0);
    private final CANcoder pivotCancoder = new CANcoder(Constants.Manipulator.CANCODER_ID);

    // Button box LED compatability
    ButtonBox buttonBox;

    public Superstructure(ButtonBox buttonBox) {
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
            .withKA(Constants.Elevator.kS)
            .withGravityType(GravityTypeValue.Elevator_Static);
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Elevator.MAX_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        // config.CurrentLimits.StatorCurrentLimit = 60;
        // config.CurrentLimits.SupplyCurrentLimit = 60;
        elevatorA.getConfigurator().apply(config);
        elevatorB.getConfigurator().apply(config);
        elevatorA.setPosition(0);
        elevatorB.setPosition(0);

        // Manipulator
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotionMagic
            .withMotionMagicCruiseVelocity(Constants.Manipulator.MAX_VELOCITY)
            .withMotionMagicAcceleration(Constants.Manipulator.MAX_ACCEL);
        pivotConfig.Slot0
            .withKP(Constants.Manipulator.P)
            .withKI(Constants.Manipulator.I)
            .withKD(Constants.Manipulator.D)
            .withKS(Constants.Manipulator.kS)
            .withKG(Constants.Manipulator.kG)
            .withKA(Constants.Manipulator.kA)
            .withKV(Constants.Manipulator.kV)
            .withGravityType(GravityTypeValue.Arm_Cosine);
        pivotConfig.Feedback
            .withRemoteCANcoder(pivotCancoder)
            .withRotorToSensorRatio(Constants.Manipulator.PIVOT_RATIO);
        pivotConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        pivotConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Manipulator.MAX_ROTATIONS;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Manipulator.MIN_ROTATIONS;
        pivot.getConfigurator().apply(pivotConfig);
        setPreset(SuperstructurePreset.STOW_LOWER);

        this.buttonBox = buttonBox;
    }

    @Override
    public void periodic() {
        // Button box LEDs
        for (Button button : Button.values()) {
            buttonBox.LED(button, button == state.button);
        }
    }

    // Set elevator position
    private void setElevator(double rotations) {
        if (rotations > 0.2 || elevatorA.getPosition().getValueAsDouble() > 0.2) {
            elevatorA.setControl(mmVoltageReq.withPosition(rotations));
            elevatorB.setControl(followReq);
        } else {
            elevatorA.set(0);
            elevatorB.set(0);
        }
    }

    private void setPivot(double rotations) {
        pivot.setControl(pivotReq.withPosition(rotations));
    }

    public Command setPreset(SuperstructurePreset preset) {
        return this.run(() -> {
            setElevator(preset.elevatorRotations);
            setPivot(preset.pivotRotations);
            beltLeft.set(preset.leftBelt);
            beltRight.set(preset.rightBelt);
            state = preset;
        });
    }

    public Command setManual(DoubleSupplier elevatorRotations, DoubleSupplier pivotRotations, DoubleSupplier leftBelt, DoubleSupplier rightBelt) {
        return this.run(() -> {
            setElevator(elevatorRotations.getAsDouble());
            setPivot(pivotRotations.getAsDouble());
            beltLeft.set(leftBelt.getAsDouble());
            beltRight.set(rightBelt.getAsDouble());
            state = SuperstructurePreset.NONE;
        });
    }

    public Command setPresetWithBeltOverride(SuperstructurePreset preset, DoubleSupplier leftBelt, DoubleSupplier rightBelt) {
        return this.run(() -> {
            // System.out.println("Moving elevator...");
            setElevator(preset.elevatorRotations);
            setPivot(preset.pivotRotations);
            beltLeft.set(leftBelt.getAsDouble());
            beltRight.set(rightBelt.getAsDouble());
            state = preset;
        });
    }

    public SuperstructurePreset getState() {
        return state;
    }

    public boolean atSetpoint() {
        return elevatorA.getClosedLoopError().getValueAsDouble() < Constants.Elevator.ROTATION_TOLERANCE
            && pivot.getClosedLoopError().getValueAsDouble() < Constants.Manipulator.ROTATION_TOLERANCE;
    }

    // SysID to find elevator gains
    // Should be safe and unlikely to damage anything IF the soft limits are working as intended.
    private final VoltageOut voltReq = new VoltageOut(0.0);

    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),        // Use default ramp rate (1 V/s)
                Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
        null,        // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                state -> { testState.append(state.toString()); }
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
