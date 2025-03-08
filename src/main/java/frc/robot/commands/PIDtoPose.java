package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Functions;

// TODO - move to inside of CommandSwerveDrivetrain subsystem for encapsulation purposes
public class PIDtoPose extends Command {
    private SwerveRequest.FieldCentric request;

    private CommandSwerveDrivetrain drivetrain;

    private PIDController xController, yController, rotateController;

    private Pose2d setpoint;
    private double maxVel = 4;

    public PIDtoPose(CommandSwerveDrivetrain drivetrain, Pose2d setpoint) {
        this.setpoint = setpoint;
        this.drivetrain = drivetrain;
        xController = new PIDController(4, 0, 0.1);
        yController = new PIDController(4, 0, 0.1);
        rotateController = new PIDController(0.2, 0, 0);
        request = new SwerveRequest.FieldCentric()
                .withDeadband(0).withRotationalDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
                .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        // rotateController.setTolerance(50);
        // xController.setTolerance(1);
        // yController.setTolerance(1);
        rotateController.reset();
        xController.reset();
        yController.reset();
    }

    @Override
    public void execute() {
        drivetrain.setControl(request
                .withVelocityX(Functions.clampMagnitude(
                        xController.calculate(drivetrain.getState().Pose.getX() - setpoint.getX()),
                        maxVel))
                .withVelocityY(Functions.clampMagnitude(
                        yController.calculate(drivetrain.getState().Pose.getY() - setpoint.getY()),
                        maxVel))
                .withRotationalRate(rotateController.calculate(
                        drivetrain.getState().Pose.getRotation().minus(setpoint.getRotation()).getDegrees())));
        System.out.println("X: " + xAtSetpoint());
        System.out.println("Y: " + yAtSetpoint());
        System.out.println("Rotate: " + rotateAtSetpoint());
        System.out.println("VelocityX: " + (drivetrain.getState().Speeds.vxMetersPerSecond < 0.1));
        System.out.println("VelocityY: " + (drivetrain.getState().Speeds.vyMetersPerSecond < 0.1));
    }

    private boolean xAtSetpoint() {
        return Math.abs(drivetrain.getState().Pose.getX() - setpoint.getX()) < 0.02;
    }

    private boolean yAtSetpoint() {
        return Math.abs(drivetrain.getState().Pose.getY() - setpoint.getY()) < 0.02;
    }

    private boolean rotateAtSetpoint() {
        return Math.abs(drivetrain.getState().Pose.getRotation().minus(setpoint.getRotation()).getDegrees()) < 2.5;
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return xAtSetpoint() && yAtSetpoint() && rotateAtSetpoint()
                && Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) < 0.1
                && Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond) < 0.1;
    }
}
