package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.lists.Constants;

public class Climb extends SubsystemBase {
    // @Logged(name = "ClimbMotor")
    private final TalonFX
        climbMotor = new TalonFX(Constants.Climb.CLIMB_ID),
        acquisitionMotor = new TalonFX(Constants.Climb.ACQUISITION_ID);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Climb.SENSOR_ID);
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    public Climb() {
        // Climb motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0
            .withKP(Constants.Climb.P)
            .withKI(Constants.Climb.I)
            .withKD(Constants.Climb.D)
            .withKS(Constants.Climb.kS)
            .withKG(Constants.Climb.kG)
            .withKV(Constants.Climb.kV)
            .withKA(Constants.Climb.kS);
        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climb.MAX_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climb.BACK_ROTATIONS;
        config.MotionMagic
            .withMotionMagicCruiseVelocity(Constants.Climb.MAX_VELOCITY_OUT)
            .withMotionMagicAcceleration(Constants.Climb.MAX_ACCEL_OUT);
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        climbMotor.getConfigurator().apply(config);
        climbMotor.setPosition(0);

        // Acquisition motor
        TalonFXConfiguration acquireConfig = new TalonFXConfiguration();
        acquireConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        acquireConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        acquisitionMotor.getConfigurator().apply(acquireConfig);
    }

    public Command set(DoubleSupplier encoder, DoubleSupplier acquireSpeed) {
        return this.run(() -> {
            climbMotor.setControl(request.withPosition(encoder.getAsDouble()));
            acquisitionMotor.set(acquireSpeed.getAsDouble());
        });
    }

    public Command setWithMotionProfile(DoubleSupplier encoder, double maxVel, double maxAccel, DoubleSupplier acquireSpeed) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                climbMotor.getConfigurator().apply(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(maxVel)
                    .withMotionMagicAcceleration(maxAccel));
            }),
            set(encoder, acquireSpeed)
        );
    }

    public Command extend() {
        return setWithMotionProfile(() -> Constants.Climb.MAX_ROTATIONS, Constants.Climb.MAX_VELOCITY_OUT, Constants.Climb.MAX_ACCEL_OUT, () -> 0.35);
    }

    public Command lift() {
        return setWithMotionProfile(() -> Constants.Climb.BACK_ROTATIONS, Constants.Climb.MAX_VELOCITY_IN, Constants.Climb.MAX_ACCEL_IN, () -> 0)
            .until(switchTriggered()::getAsBoolean)
            .finallyDo(() -> climbMotor.set(0));
    }

    public Command retract() {
        return setWithMotionProfile(() -> 0, Constants.Climb.MAX_VELOCITY_IN, Constants.Climb.MAX_ACCEL_IN, () -> 0)
            .until(switchTriggered()::getAsBoolean)
            .finallyDo(() -> climbMotor.set(0));
    }

    public Trigger switchTriggered() {
        return new Trigger(limitSwitch::get);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Limit Switch", switchTriggered()::getAsBoolean, null);
    }
}
