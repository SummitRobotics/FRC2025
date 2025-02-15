package frc.robot.commands;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scrubber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.lists.Constants;

public class AutoPickup extends SequentialCommandGroup {

    public enum CoralStationSide {
        LEFT("LeftStation"),
        RIGHT("RightStation");
        public String pathName;
        CoralStationSide(String pathName) {
            this.pathName = pathName;
        }
    }

    // For auto selector (maybe should be someplace else)
    public enum AutoSegment {
        DO_FIRST,
        DO_SECOND,
        DO_THIRD
    }

    public static CoralStationSide getCoralSide(Pose2d pose) {
        if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
            return pose.getY() > 4 ? CoralStationSide.LEFT : CoralStationSide.RIGHT;
        } else {
            return pose.getY() < 4 ? CoralStationSide.LEFT : CoralStationSide.RIGHT;
        }
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            3.0 / 2, 4.0 / 2,
            Units.degreesToRadians(270 / 2), Units.degreesToRadians(360 / 2));
    public AutoPickup(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, CoralStationSide side) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(side.pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }
        if (!Utils.isSimulation()) {
            addCommands(
                new ParallelCommandGroup(
                    scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations),
                    superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE),
                    AutoBuilder.pathfindThenFollowPath(path, constraints)
                ).until(superstructure.getCoralSensorIntake().and(superstructure.getCoralSensorPlace()))
            );
        } else {
            addCommands(AutoBuilder.pathfindThenFollowPath(path, constraints));
        }
    }

    // TODO - finish / tune
    public static Command timeOfFlightAlign(CommandSwerveDrivetrain drivetrain, DoubleSupplier leftToF, DoubleSupplier rightToF) {
        SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        return drivetrain.applyRequest(() -> request.withVelocityX(0).withRotationalRate(0));
    }
}
