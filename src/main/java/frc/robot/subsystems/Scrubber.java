package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Constants;

public class Scrubber extends SubsystemBase {

    // @Logged(name = "ScrubMotor")
    private final TalonFX scrubber = new TalonFX(Constants.Scrubber.SCRUBBER_ID);
    private final PositionVoltage scrubberReq = new PositionVoltage(0).withSlot(0);
    private DoubleSupplier manipulatorPivot;

    public Scrubber(DoubleSupplier manipulatorPivot) {
        this.manipulatorPivot = manipulatorPivot;
        TalonFXConfiguration scrubberConfig = new TalonFXConfiguration();
        scrubberConfig.Slot0
            .withKP(Constants.Scrubber.P)
            .withKI(Constants.Scrubber.I)
            .withKD(Constants.Scrubber.D);
        scrubberConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        scrubberConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        scrubberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        scrubberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        scrubberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Scrubber.MAX_ROTATIONS;
        scrubberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations;
        scrubber.getConfigurator().apply(scrubberConfig);
        scrubber.setPosition(Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations);
    }

    public Command set(DoubleSupplier target) {
        // Cannot exceed position of the manipulator; it'd otherwise collide.
        return this.run(() -> scrubber.setControl(scrubberReq.withPosition(
            Math.min(target.getAsDouble(), Constants.Scrubber.GEAR_RATIO * manipulatorPivot.getAsDouble())
        )));
    }

    public Command flick() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(1), set(() -> Constants.Scrubber.MAX_ROTATIONS)),
            set(() -> 0)
        );
    }

    public boolean safe() {
        return Functions.withinTolerance(scrubber.getPosition().getValueAsDouble(), Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations, 2.5);
    }
}
