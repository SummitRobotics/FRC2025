package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        // 4 meters is the midline of the field
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

    private AlignRequestToF alignRequest;

    public AutoPickup(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, CoralStationSide side) {
        alignRequest = new AlignRequestToF(superstructure.getToFLeft(), superstructure.getToFRight());
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
                    new SequentialCommandGroup(
                        AutoBuilder.pathfindThenFollowPath(path, constraints),
                        drivetrain.applyRequest(() -> alignRequest).until(() -> alignRequest.done())
                    )
                ).until(superstructure.getCoralSensorIntake().and(superstructure.getCoralSensorPlace()))
            );
        } else {
            addCommands(AutoBuilder.pathfindThenFollowPath(path, constraints));
        }
    }
}
