package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.lists.Constants;

public class Climb extends SubsystemBase {
    private final TalonFX
        climbMotor = new TalonFX(Constants.Climb.CLIMB_ID);
    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0
            .withKP(Constants.Climb.P)
            .withKI(Constants.Climb.I)
            .withKD(Constants.Climb.D)
            .withKS(Constants.Climb.kS)
            .withKG(Constants.Climb.kG)
            .withKV(Constants.Climb.kV)
            .withKA(Constants.Climb.kS);
        climbMotor.getConfigurator().apply(config);
    }

    public Command set(DoubleSupplier encoder) {
        return this.run(() -> climbMotor.setControl(request.withPosition(encoder.getAsDouble())));
    }
}
