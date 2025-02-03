package frc.robot.utilities;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

// CTRE Logger so we don't need to deal with Pro licensing and converting .hoots with SignalLogger
// See https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html#logging-third-party-data
@CustomLoggerFor(TalonFX.class)
public class LogTalon extends ClassSpecificLogger<TalonFX> {
    public LogTalon() {
        super(TalonFX.class);
    }

    @Override
    protected void update(EpilogueBackend backend, TalonFX motor) {
        backend.log("Velocity", motor.getVelocity().getValueAsDouble());
        backend.log("Voltage", motor.getMotorVoltage().getValueAsDouble());
        backend.log("Position", motor.getPosition().getValueAsDouble());
    }
}
