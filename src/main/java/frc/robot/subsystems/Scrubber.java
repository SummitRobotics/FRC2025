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
import frc.robot.utilities.lists.Constants;

public class Scrubber extends SubsystemBase {

    private final TalonFX scrubber = new TalonFX(Constants.Scrubber.SCRUBBER_ID);
    private final PositionVoltage scrubberReq = new PositionVoltage(0).withSlot(0);
    // private final ArmFeedforward scrubberFF = new ArmFeedforward(Constants.Scrubber.kS, Constants.Scrubber.kG, Constants.Scrubber.kV);

    public Scrubber() {
        TalonFXConfiguration scrubberConfig = new TalonFXConfiguration();
        scrubberConfig.Slot0
            .withKP(Constants.Scrubber.P)
            .withKI(Constants.Scrubber.I)
            .withKD(Constants.Scrubber.D);
        scrubberConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        scrubberConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        scrubber.getConfigurator().apply(scrubberConfig);
    }

    public Command set(DoubleSupplier encoder) {
        return this.run(() -> scrubber.setControl(scrubberReq
            .withPosition(encoder.getAsDouble())
            .withLimitForwardMotion(scrubber.getPosition().getValueAsDouble() > Constants.Scrubber.MAX_ROTATIONS)
            .withLimitReverseMotion(scrubber.getPosition().getValueAsDouble() < 0)
        ));
    }

    public Command flick() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(new WaitCommand(1), set(() -> Constants.Scrubber.MAX_ROTATIONS)),
            set(() -> 0)
        );
    }
}
