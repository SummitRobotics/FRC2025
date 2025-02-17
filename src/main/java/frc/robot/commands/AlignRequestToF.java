package frc.robot.commands;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.utilities.lists.Constants;

public class AlignRequestToF implements SwerveRequest {
    private DoubleSupplier leftToF;
    private DoubleSupplier rightToF;
    private PIDController translateController = new PIDController(0, 0, 0);
    private PIDController rotateController = new PIDController(5, 0, 0);
    SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public AlignRequestToF(DoubleSupplier leftToF, DoubleSupplier rightToF) {
        this.leftToF = leftToF;
        this.rightToF = rightToF;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        // If measurements are invalid, the request should do nothing.
        return request
            .withVelocityX(translateController.calculate(leftToF.getAsDouble() > 0 && rightToF.getAsDouble() > 0 ? Math.min(
                leftToF.getAsDouble(), rightToF.getAsDouble()) : Constants.Physical.TOF_OFFSET_METERS, Constants.Physical.TOF_OFFSET_METERS
            ))
            .withRotationalRate(rotateController.calculate(leftToF.getAsDouble() > 0 && rightToF.getAsDouble() > 0
                ? leftToF.getAsDouble() - rightToF.getAsDouble() : 0, 0))
            .apply(parameters, modulesToApply);
    }

    public boolean done() {
        // TODO - check tolerances
        return translateController.atSetpoint() && rotateController.atSetpoint();
    }
}
