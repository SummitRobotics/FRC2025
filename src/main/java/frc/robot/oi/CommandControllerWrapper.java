package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * CommandControllerWrapper is a wrapper class that abstracts the differences between
 * CommandXboxController and CommandPS5Controller, using XBox naming conventions.
 */
public class CommandControllerWrapper {
    private final CommandXboxController xboxController;
    private final CommandPS5Controller ps5Controller;

    public CommandControllerWrapper(CommandXboxController xboxController) {
        this.xboxController = xboxController;
        this.ps5Controller = null;
    }

    public CommandControllerWrapper(CommandPS5Controller ps5Controller) {
        this.xboxController = null;
        this.ps5Controller = ps5Controller;
    }

    public Trigger a() {
        return xboxController != null ? xboxController.a() : ps5Controller.cross();
    }

    public Trigger b() {
        return xboxController != null ? xboxController.b() : ps5Controller.circle();
    }

    public Trigger x() {
        return xboxController != null ? xboxController.x() : ps5Controller.square();
    }

    public Trigger y() {
        return xboxController != null ? xboxController.y() : ps5Controller.triangle();
    }

    public Trigger leftBumper() {
        return xboxController != null ? xboxController.leftBumper() : ps5Controller.L1();
    }

    public Trigger rightBumper() {
        return xboxController != null ? xboxController.rightBumper() : ps5Controller.R1();
    }

    public Trigger start() {
        return xboxController != null ? xboxController.start() : ps5Controller.options();
    }

    public Trigger back() {
        return xboxController != null ? xboxController.back() : ps5Controller.create();
    }
    
    public double getLeftTriggerAxis() {
        return xboxController != null ? xboxController.getLeftTriggerAxis() : ps5Controller.getL2Axis();
    }

    public double getRightTriggerAxis() {
        return xboxController != null ? xboxController.getRightTriggerAxis() : ps5Controller.getR2Axis();
    }

    public double getLeftX() {
        return xboxController != null ? xboxController.getLeftX() : ps5Controller.getLeftX();
    }

    public double getLeftY() {
        return xboxController != null ? xboxController.getLeftY() : ps5Controller.getLeftY();
    }

    public double getRightX() {
        return xboxController != null ? xboxController.getRightX() : ps5Controller.getRightX();
    }

    public double getRightY() {
        return xboxController != null ? xboxController.getRightY() : ps5Controller.getRightY();
    }

    // Add other methods as needed...
}
