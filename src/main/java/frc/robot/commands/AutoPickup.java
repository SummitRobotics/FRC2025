package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
        // ConditionalCommand pathfind = 
            // new ConditionalCommand(
                // AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                // AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                // () -> side.get() == CoralStationSide.LEFT
            // );
        if (!Utils.isSimulation()) {
            addCommands(
                new ParallelCommandGroup(
                    // new PrintCommand("Auto pickup running").repeatedly(),
                    scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations),
                    new SequentialCommandGroup(
                        superstructure.setPresetWithAutoCenter(SuperstructurePreset.STOW_UPPER).withDeadline(new WaitCommand(1)),
                        superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE)
                    ),
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new ConditionalCommand(
                                AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                                AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                                () -> side.get() == CoralStationSide.LEFT
                            ),
                            // pathfind.withTimeout(3).repeatedly(),
                            new PrintCommand("Pathfinding to station").repeatedly()
                        )
                        // new AlignRequestToF(drivetrain, superstructure.getToFLeft(), superstructure.getToFRight())
                    ).withName("Pathfinding to station"),
                    new WaitCommand(1),
                    new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                ).withDeadline(
                    new WaitUntilCommand(() -> superstructure.getCoralSensorIntake().and(superstructure.getCoralSensorPlace()).getAsBoolean())
                ),
                // new ConditionalCommand(
                //    new InstantCommand(() -> drivetrain.resetPose(leftPath.getStartingDifferentialPose())),
                //    new InstantCommand(() -> drivetrain.resetPose(rightPath.getStartingDifferentialPose())),
                //    () -> side.get() == CoralStationSide.LEFT
                // ),
                //.until(() -> superstructure.getCoralSensorIntake().debounce(0.2).getAsBoolean() && superstructure.getCoralSensorPlace().debounce(0.2).getAsBoolean()),
                new PrintCommand("Finished auto align"),
                // new PrintCommand("Actually finished auto align"),
                // Back up slightly
                new ParallelCommandGroup(
                    superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE),
                    new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-2))).repeatedly()
                ).withDeadline(new WaitCommand(0.25)),
                new PrintCommand("Drove backwards")
                // new AlignRequestToF(drivetrain, superstructure.getToFLeft(), superstructure.getToFRight(), 1).withDeadline(new WaitCommand(1))
            );
        } else {
            addCommands(
                new ConditionalCommand(
                    AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                    AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                    () -> side.get() == CoralStationSide.LEFT
                ),
                new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-2))).repeatedly().withDeadline(new WaitCommand(0.25))
            );
        }
    }
}
