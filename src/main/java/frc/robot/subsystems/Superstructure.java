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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.devices.CandiCoralSensor;
import frc.robot.devices.SerialTOFSensor;
import frc.robot.devices.SerialTOFSensor.SensorData;
import frc.robot.devices.SerialTOFSensor.Status;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.ButtonBox.Button;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Constants;

public class Superstructure extends SubsystemBase {

    // State machine with encoder presets
    // Those with "GO" in the name are the spitting states
    // Those with "B" in the name are the backwards placement states
    public static enum SuperstructurePreset {
        STOW_LOWER(0.1, Constants.Manipulator.MIN_ROTATIONS, 0, 0, "Stow Lower", Button.STOW_LOWER_PRESET),
        STOW_UPPER(0.1, 0.24, 0, 0, "Stow Upper", Button.STOW_UPPER_PRESET),
        RECEIVE(3.4298038440, -0.165, 0, 0, "Receive", Button.RECEIVE_PRESET),
        L1(0.1, Constants.Manipulator.MIN_ROTATIONS + 0.025, 0, 0, "1", Button.L1_PRESET),
        L2(0.1, 0.101562, 0, 0, "2", Button.L2_PRESET),
        L3(4.193848, 0.137075, 0, 0, "3", Button.L3_PRESET),
        L3_SCRUB(5.3, STOW_UPPER.pivotRotations, 0, 0, "L3 Scrub", null),
        L3B(7.637207, 0.343975, 0, 0, "3B", null),
        L4(14.85209, 0.08333, 0, 0, "4", Button.L4_PRESET),
        L4B(15.7, 0.325, 0, 0, "4B", Button.L4_BACKWARDS),
        L1_GO(L1.elevatorRotations, L1.pivotRotations, -1, -1, "1", null),
        L2_GO(L2.elevatorRotations, L2.pivotRotations, 0.85, 0.85, "2", null),
        L3_GO(L3.elevatorRotations, L3.pivotRotations, 0.85, 0.85, "3", null),
        L3B_GO(L3B.elevatorRotations, L3B.pivotRotations, -1, -1, "4B", null),
        L4_GO(L4.elevatorRotations, L4.pivotRotations, 1, 1, "4", null),
        L4B_GO(L4B.elevatorRotations, L4B.pivotRotations, -1, -1, "4B", null),
        L4_INTERMEDIATE(L3.elevatorRotations + 2, STOW_UPPER.pivotRotations, 0, 0, "Intermediate", null),
        CLIMB_STOW(0.1, 0.125, 0, 0, "Stow Climb", null),
        MANUAL_OVERRIDE(0.3, 0, 0, 0, "None", null); // being manually overridden to something
        public double elevatorRotations;
        public double pivotRotations;
        public double leftBelt;
        public double rightBelt;
        public String description;
        public Button button;
        SuperstructurePreset(double elevatorRotations, double pivotRotations, double leftBelt, double rightBelt, String description, Button button) {
            this.elevatorRotations = elevatorRotations;
            this.pivotRotations = pivotRotations;
            this.leftBelt = leftBelt;
            this.rightBelt = rightBelt;
            this.description = description;
            this.button = button;
        }

        public static SuperstructurePreset getCorrespondingGoState(SuperstructurePreset preset) {
            switch (preset) {
                case L1: return L1_GO;
                case L2: return L2_GO;
                case L3: return L3_GO;
                case L3B: return L3B_GO;
                case L4: return L4_GO;
                case L4B: return L4B_GO;
                default: return preset;
            }
        }

        public static SuperstructurePreset getCorrespondingBackwardsState(SuperstructurePreset preset) {
            switch (preset) {
                case L3: return L3B;
                case L4: return L4B;
                default: return preset;
            }
        }

        public static SuperstructurePreset getCorrespondingBackwardsGo(SuperstructurePreset preset) {
            return getCorrespondingGoState(getCorrespondingBackwardsState(preset));
        }
    }

    private SuperstructurePreset state = SuperstructurePreset.STOW_LOWER;

    // Elevator
    // @Logged(name = "ElevatorA")
    private final TalonFX
        elevatorA = new TalonFX(Constants.Elevator.ELEVATOR_ID_A);
    // @Logged(name = "ElevatorB")
    private final TalonFX
        elevatorB = new TalonFX(Constants.Elevator.ELEVATOR_ID_B);
    // StringLogEntry testState = new StringLogEntry(DataLogManager.getLog(), "testState");
    private final MotionMagicVoltage mmVoltageReq = new MotionMagicVoltage(0).withSlot(0);
    private final Follower followReq = new Follower(Constants.Elevator.ELEVATOR_ID_A, false);
    // Manipulator
    private final TalonFX
        beltLeft = new TalonFX(Constants.Manipulator.BELT_LEFT_ID),
        beltRight = new TalonFX(Constants.Manipulator.BELT_RIGHT_ID);
    // @Logged(name = "Pivot")
    private final TalonFX pivot = new TalonFX(Constants.Manipulator.PIVOT_ID);
    private final MotionMagicVoltage pivotReq = new MotionMagicVoltage(0).withSlot(0);
    private final CANcoder pivotCancoder = new CANcoder(Constants.Manipulator.CANCODER_ID);
    private boolean pivotSafe = false;
    private boolean elevatorSafe = false;
    // Sensor
    // @Logged(name = "ManipulatorSensors")
    private CandiCoralSensor coralSensor = new CandiCoralSensor();
    private SerialTOFSensor tofSensor = new SerialTOFSensor(230400);

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
        config.CurrentLimits.StatorCurrentLimit = 90;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
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

        // Belts
        TalonFXConfiguration beltConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        beltLeft.getConfigurator().apply(beltConfig);
        beltRight.getConfigurator().apply(beltConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

        setPreset(SuperstructurePreset.STOW_LOWER);

        this.buttonBox = buttonBox;

        // Questionable non-feedback re-homing solution (assume it's fallen to the bottom after 2 seconds of being commanded down)
        new Trigger(() -> state.elevatorRotations < 0.2).debounce(2).onTrue(new InstantCommand(() -> {
            elevatorA.setPosition(0);
            elevatorB.setPosition(0);
        }));
    }

    public Trigger getCoralSensorIntake() {
        return coralSensor.detectedIntakeSide();
    }

    public Trigger getCoralSensorPlace() {
        return coralSensor.detectedPlacementSide();
    }

    public DoubleSupplier getToFLeft() {
        return () -> {
            SensorData data = tofSensor.getSensorData(Constants.Devices.TOF_ID_LEFT);
            return data != null && data.status == Status.MEASURE_VALID ? data.distance : -1;
        };
    }

    public DoubleSupplier getToFRight() {
        return () -> {
            SensorData data = tofSensor.getSensorData(Constants.Devices.TOF_ID_RIGHT);
            return data != null && data.status == Status.MEASURE_VALID ? data.distance : -1;
        };
    }

    @Override
    public void periodic() {
        // Button box LEDs
        for (Button button : Button.values()) {
            buttonBox.LED(button, button == state.button);
        }
        tofSensor.tick();
    }

    // @Logged(name = "Command")
    public String getCurrentCommandName() {
        return (getCurrentCommand() != null) ? getCurrentCommand().getName() : "";
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

    private Command set(DoubleSupplier elevatorRotations, DoubleSupplier pivotRotations, DoubleSupplier leftBelt, DoubleSupplier rightBelt, boolean isManual) {
        if (isManual) state = SuperstructurePreset.MANUAL_OVERRIDE;
        return this.run(() -> {

            // Do not collide mechanisms
            pivotSafe =
                pivotCancoder.getPosition().getValueAsDouble() > SuperstructurePreset.STOW_UPPER.pivotRotations - Constants.Manipulator.ROTATION_TOLERANCE;
            elevatorSafe = 
                Functions.withinTolerance(elevatorA.getPosition().getValueAsDouble(), elevatorRotations.getAsDouble(), Constants.Elevator.ROTATION_TOLERANCE * 1.5);
            // Freeze the elevator until pivot's safe
            setElevator(pivotSafe ? elevatorRotations.getAsDouble() : elevatorA.getPosition().getValueAsDouble());
            // Save the pivot until elevator's positioned
            setPivot(elevatorSafe ? pivotRotations.getAsDouble() : SuperstructurePreset.STOW_UPPER.pivotRotations);
            beltLeft.set(leftBelt.getAsDouble());
            beltRight.set(rightBelt.getAsDouble());
        });
    }

    public Command setManual(DoubleSupplier elevatorRotations, DoubleSupplier pivotRotations, DoubleSupplier leftBelt, DoubleSupplier rightBelt) {
        return set(elevatorRotations, pivotRotations, leftBelt, rightBelt, true);
    }

    public Command setPresetWithBeltOverride(SuperstructurePreset preset, DoubleSupplier leftBelt, DoubleSupplier rightBelt) {
        return set(() -> preset.elevatorRotations, () -> preset.pivotRotations, leftBelt, rightBelt, false).alongWith(new InstantCommand(() -> {
            state = preset;
            // System.out.println("State:" + getState().description);
        }).repeatedly());
    }

    // Use sensors to automatically hold the note in the middle
    public Command setPresetWithAutoCenter(SuperstructurePreset preset) {
        DoubleSupplier beltSupplier = new DoubleSupplier() {
            boolean detected = false;
            boolean backedUp = false;
            @Override
            public double getAsDouble() {
                if (getCoralSensorIntake().negate().getAsBoolean()) {
                    detected = false;
                    backedUp = false;
                }
                if (!detected) {
                    if (getCoralSensorPlace().negate().getAsBoolean()) return 0.55;
                    if (getCoralSensorIntake().negate().getAsBoolean()) return -0.55;
                    detected = true;
                    return 0;
                } else {
                    if (!backedUp) {
                        // back until we lose the sensor
                        if (getCoralSensorPlace().negate().getAsBoolean()) {
                            backedUp = true;
                            return 0;
                        }
                        return -0.05;
                    } else {
                        // Forward until we gain the sensor again
                        if (getCoralSensorPlace().negate().getAsBoolean()) return 0.05;
                        return 0;
                    }
                }
            }
        };
        return setPresetWithBeltOverride(preset, beltSupplier, beltSupplier);
    }

    public Command setPresetRockBackwards(SuperstructurePreset preset) {
        DoubleSupplier beltSupplier = () -> {
            if (getCoralSensorIntake().getAsBoolean()) return 0.2;
            return 0;
        };

        return setPresetWithBeltOverride(preset, beltSupplier, beltSupplier);
    }

    public Command setPreset(SuperstructurePreset preset) {
        return setPresetWithBeltOverride(preset, () -> preset.leftBelt, () -> preset.rightBelt);
    }

    public SuperstructurePreset getState() {
        return state;
    }

    public boolean atSetpoint() {
        return pivotSafe && elevatorSafe && elevatorA.getClosedLoopError().getValueAsDouble() < Constants.Elevator.ROTATION_TOLERANCE
            && pivot.getClosedLoopError().getValueAsDouble() < Constants.Manipulator.ROTATION_TOLERANCE;// && pivot.getVelocity().getValueAsDouble() < 0.005;
    }

    public double pivotRotations() {
        return pivotCancoder.getPosition().getValueAsDouble();
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
                state -> { /*testState.append(state.toString());*/ }
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", getState()::name, null);
        builder.addBooleanProperty("Sensor Intake", getCoralSensorIntake()::getAsBoolean, null);
        builder.addBooleanProperty("Sensor Place", getCoralSensorPlace()::getAsBoolean, null);
        builder.addDoubleProperty("ToF Left", getToFLeft()::getAsDouble, null);
        builder.addDoubleProperty("ToF Right", getToFRight()::getAsDouble, null);
        builder.addDoubleProperty("Elevator encoder", elevatorA.getPosition()::getValueAsDouble, null);
        builder.addStringProperty("Command", () -> this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "", null);
    }
}
