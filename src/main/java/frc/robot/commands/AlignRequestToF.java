package frc.robot.commands;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.lists.Constants;

// TODO - move to inside of CommandSwerveDrivetrain subsystem for encapsulation purposes
public class AlignRequestToF extends Command {
    private DoubleSupplier leftToF;
    private DoubleSupplier rightToF;
    private PIDController translateController;
    private PIDController rotateController;
    SwerveRequest.RobotCentric request;

    CommandSwerveDrivetrain drivetrain;

    public AlignRequestToF(CommandSwerveDrivetrain drivetrain, DoubleSupplier leftToF, DoubleSupplier rightToF) {
        this.drivetrain = drivetrain;
        this.leftToF = leftToF;
        this.rightToF = rightToF;
        translateController = new PIDController(4, 0, 0.1);
        rotateController = new PIDController(8, 0, 0.2);
        request = new SwerveRequest.RobotCentric()
            .withDeadband(0).withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        rotateController.setTolerance(0.02);
        translateController.setTolerance(0.001);
        rotateController.reset();
        translateController.reset();
    }

    @Override
    public void execute() {
        // If measurements are invalid, the request should do nothing.
        // System.out.println("Aligning!");
        // System.out.println("Distance delta" + (leftToF.getAsDouble() - rightToF.getAsDouble()));
        // System.out.println("Rotation rate:" + getRotationRate());
        drivetrain.setControl(request
            .withVelocityX(getTranslationRate())
            // .withRotationalRate(getRotationRate())
        );
    }

    private double getRotationRate() {
        return rotateController.atSetpoint() || leftToF.getAsDouble() < 0 || rightToF.getAsDouble() < 0 ? 0
            : rotateController.calculate(leftToF.getAsDouble() - rightToF.getAsDouble(), 0);
    }

    private double getTranslationRate() {
        return leftToF.getAsDouble() > 0 && rightToF.getAsDouble() > 0 ? -translateController.calculate(Math.min(
            leftToF.getAsDouble(), rightToF.getAsDouble()), Constants.Physical.TOF_OFFSET_METERS) : 0;
    }

    @Override
    public boolean isFinished() {
        return translateController.atSetpoint() && rotateController.atSetpoint();
    }
}
