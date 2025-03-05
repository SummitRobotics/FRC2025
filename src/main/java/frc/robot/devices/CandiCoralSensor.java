package frc.robot.devices;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utilities.lists.Constants;

public class CandiCoralSensor {
    CANdi candi = new CANdi(Constants.Manipulator.CORAL_SENSOR_ID);

    public CandiCoralSensor() {
        CANdiConfiguration config = new CANdiConfiguration();
        candi.getConfigurator().apply(config);
    }

    public Trigger detectedIntakeSide() {
        return new Trigger(() -> candi.getS1State().getValueAsDouble() == 2.0);
    }

    public Trigger detectedPlacementSide() {
        return new Trigger(() -> candi.getS2State().getValueAsDouble() == 2.0);
    }

    @Logged(name = "IntakeSensor")
    public boolean detectedIntakeSideBool() {
        return candi.getS1State().getValueAsDouble() == 2.0;
    }

    @Logged(name = "PlacementSensor")
    public boolean detectedPlacementSideBool() {
        return candi.getS2State().getValueAsDouble() == 2.0;
    }
}
