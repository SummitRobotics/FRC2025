package frc.robot.commands;

import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
        NONE("None"),
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
        this(drivetrain, superstructure, scrubber, side, "");
    }

    public AutoPickup(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, Supplier<CoralStationSide> side, String suppliedPathName) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(270), Units.degreesToRadians(360));
        // Load the station side paths from the path planner
        PathPlannerPath leftPath;
        PathPlannerPath rightPath;
        try {
            leftPath = PathPlannerPath.fromPathFile(CoralStationSide.LEFT.pathName);
            rightPath = PathPlannerPath.fromPathFile(CoralStationSide.RIGHT.pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }
        // If given a supplied path name then load it
        PathPlannerPath suppliedPath;
        if (!suppliedPathName.isEmpty()) {
            try {
                suppliedPath = PathPlannerPath.fromPathFile(suppliedPathName);
            } catch (Exception e) {
                throw new RuntimeException();
            }
        } else {
            // Set to leftPath if no supplied path name is given, but otherwise unused
            suppliedPath = leftPath;
        }
        // ConditionalCommand pathfind = 
            // new ConditionalCommand(
                // AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                // AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                // () -> side.get() == CoralStationSide.LEFT
            // );
        if (!Utils.isSimulation()) {
            addCommands(
                new ConditionalCommand(
                    // Perform the auto pickup sequence until the coral sensor is triggered
                    new ParallelCommandGroup(
                        // Stow the scrubber
                        scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations),
                        // Move the superstructure to the stow position for some time, then to receive position; all while driving the coral intake
                        new SequentialCommandGroup(
                            superstructure.setPresetWithAutoCenter(SuperstructurePreset.STOW_UPPER).withTimeout(0/*1.25*/),
                            superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE)
                        ),
                        // Drive to the station until the coral sensor is triggered
                        // Follow the suppplied path if given, otherwise pathfind and then follow the left or right path
                        new SequentialCommandGroup(
                            new ConditionalCommand(
                                new ConditionalCommand(
                                    AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                                    AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                                    () -> side.get() == CoralStationSide.LEFT
                                ),
                                AutoBuilder.followPath(suppliedPath),
                                () -> suppliedPathName.isEmpty()
                            ),
                            // Path may end with a target velocity, to drive robot into station, do so for a short time
                            new WaitCommand(0.5),
                            // Stop the robot from moving into station, the robot will move backwards after coral intake so OK if never reach this
                            new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)))
                        )
                    ).withDeadline(
                        // Wait until the coral intake sensor is triggered (the coral place sensor may not be triggered)
                        new WaitUntilCommand(() -> superstructure.getCoralSensorIntake().and(superstructure.getCoralSensorPlace()).getAsBoolean())
                    ),
                    // Do nothing if given the flag
                    new InstantCommand(() -> {}),
                    () -> side.get() != CoralStationSide.NONE
                )
            );
        } else {
            // Simulation is simplified pickup
            addCommands(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // Drive to the station
                        new ConditionalCommand(
                            new ConditionalCommand(
                                AutoBuilder.pathfindThenFollowPath(leftPath, constraints),
                                AutoBuilder.pathfindThenFollowPath(rightPath, constraints),
                                () -> side.get() == CoralStationSide.LEFT
                            ),
                            AutoBuilder.followPath(suppliedPath),
                            () -> suppliedPath != null
                        )
                    ),
                    // Do nothing if given the flag
                    new InstantCommand(() -> {}),
                    () -> side.get() != CoralStationSide.NONE
                )
            );
        }
    }
}
