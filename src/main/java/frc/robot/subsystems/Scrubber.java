package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Constants;

public class Scrubber extends SubsystemBase {
    public static enum ScrubberPreset {
        DOWN(0),
        UP(Constants.Scrubber.MAX_ROTATIONS),
        NONE(0);
        public double encoder; // May be switched to radians instead of rotations if we're facilitating doing feedforward here (which is probably unnecessary)
        ScrubberPreset(double encoder) {
            this.encoder = encoder;
        }
    }

    private final SparkFlex scrubber = new SparkFlex(Constants.Scrubber.SCRUBBER_ID, MotorType.kBrushless);
    // private final ArmFeedforward scrubberFF = new ArmFeedforward(Constants.Scrubber.kS, Constants.Scrubber.kG, Constants.Scrubber.kV);

    public Scrubber() {
        SparkFlexConfig scrubberConfig = new SparkFlexConfig();
        scrubberConfig.closedLoop.p(Constants.Scrubber.P).i(Constants.Scrubber.I).d(Constants.Scrubber.D);
        scrubber.configure(scrubberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command setPreset(ScrubberPreset preset) {
        return this.run(() -> scrubber.getClosedLoopController().setReference(preset.encoder, ControlType.kPosition));
    }

    public Command setManual(double encoder) {
        return this.run(() -> scrubber.getClosedLoopController().setReference(encoder, ControlType.kPosition));
    }
}
