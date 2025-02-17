package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scrubber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.lists.Constants;

public class AutoPickup extends SequentialCommandGroup {

    public static enum CoralStationSide {
        LEFT("LeftStation"),
        RIGHT("RightStation");
        public String pathName;
        CoralStationSide(String pathName) {
            this.pathName = pathName;
        }
    }

    // For auto selector (maybe should be someplace else)
    public static enum AutoSegment {
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


    public AutoPickup(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, Supplier<CoralStationSide> side) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0 / 2, 4.0 / 2,
                Units.degreesToRadians(270 / 2), Units.degreesToRadians(360 / 2));
        PathPlannerPath leftPath;
        PathPlannerPath rightPath;
        try {
            leftPath = PathPlannerPath.fromPathFile(CoralStationSide.LEFT.pathName);
            rightPath = PathPlannerPath.fromPathFile(CoralStationSide.RIGHT.pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }
        if (!Utils.isSimulation()) {
            addCommands(
                new InstantCommand(() -> {

                }),
                new ParallelCommandGroup(
                    // new InstantCommand(() -> {
                        // System.out.println("Side name " + side.get().pathName);
                        // System.out.println("Path " + path.name);
                    // }),
                    scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations),
                    superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE),
                    new SequentialCommandGroup(
                        AutoBuilder.pathfindThenFollowPath(side.get() == CoralStationSide.LEFT ? leftPath : rightPath, constraints),
                        new AlignRequestToF(drivetrain, superstructure.getToFLeft(), superstructure.getToFRight())
                    )
                ).until(superstructure.getCoralSensorIntake().and(superstructure.getCoralSensorPlace()))
            );
        } else {
            addCommands(AutoBuilder.pathfindThenFollowPath(side.get() == CoralStationSide.LEFT ? leftPath : rightPath, constraints));
        }
    }
}
