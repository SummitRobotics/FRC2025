package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scrubber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.lists.Constants;

public class AutoPlace extends SequentialCommandGroup {

    public enum HexSide {
        // Technically pathfindThenFollowPath makes manually putting in these waypoints unnecessary; I didn't know that at the time of copying them over.
        // ONE_BLUE(new Pose2d(2.678, 4.058, Rotation2d.fromDegrees(0)), "1"),
        // TWO_BLUE(new Pose2d(3.565, 2.457, Rotation2d.fromDegrees(60)), "2"),
        // THREE_BLUE(new Pose2d(5.414, 2.494, Rotation2d.fromDegrees(120)), "3"),
        // FOUR_BLUE(new Pose2d(6.296, 4.032, Rotation2d.fromDegrees(180)), "4"),
        // FIVE_BLUE(new Pose2d(5.378, 5.594, Rotation2d.fromDegrees(240)), "5"),
        // SIX_BLUE(new Pose2d(3.625, 5.601, Rotation2d.fromDegrees(300)), "6"),
        // ONE_RED(new Pose2d(8.593 + 6.296, 4.032, Rotation2d.fromDegrees(180)), "1"),
        // TWO_RED(new Pose2d(8.593 + 5.378, 5.594, Rotation2d.fromDegrees(240)), "2"),
        // THREE_RED(new Pose2d(8.593 + 3.625, 5.601, Rotation2d.fromDegrees(300)), "3"),
        // FOUR_RED(new Pose2d(8.593 + 2.678, 4.058, Rotation2d.fromDegrees(0)), "4"),
        // FIVE_RED(new Pose2d(8.593 + 3.565, 2.457, Rotation2d.fromDegrees(60)), "5"),
        // SIX_RED(new Pose2d(8.593 + 5.414, 2.494, Rotation2d.fromDegrees(120)), "6");
        ONE("1"),
        TWO("2"),
        THREE("3"),
        FOUR("4"),
        FIVE("5"),
        SIX("6");

        // public Pose2d waypoint;
        public String name;
        private HexSide(/*Pose2d waypoint,*/ String name) {
            // this.waypoint = waypoint;
            this.name = name;
        }
    }

    public enum Side {
        LEFT("L"),
        RIGHT("R");
        public String name;
        private Side(String name) {
            this.name = name;
        }
    }

    public static class Node {
        public SuperstructurePreset l;
        public Side side;
        public HexSide hexSide;
        public SuperstructurePreset scrub;
        public Node(SuperstructurePreset l, HexSide hexSide, Side side, SuperstructurePreset scrub) {
            this.l = l;
            this.side = side;
            this.hexSide = hexSide;
            this.scrub = scrub;
        }

        public String toString() {
            return "Hex: " + hexSide.name + ", Side: " + side.name + ", L: " + l.name;
        }
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            3.0 / 2, 4.0 / 2,
            Units.degreesToRadians(270 / 2), Units.degreesToRadians(360 / 2));

    public AutoPlace(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, Node node) {
        PathPlannerPath path;
        String pathName = "";
        // Name format is [side number][L/R] (e.g. 4R)
        pathName += node.hexSide.name;
        pathName += node.side.name;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }

        // Command move = AutoBuilder.pathfindThenFollowPath(path, constraints);
        Command move = new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                AutoBuilder.pathfindToPoseFlipped(path.getStartingHolonomicPose().get(), constraints),
                new ConditionalCommand(superstructure.setPreset(node.l), new InstantCommand(), () -> node.l == SuperstructurePreset.L2)
            ),
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path),
                superstructure.setPreset(
                    node.l != SuperstructurePreset.MANUAL_OVERRIDE ? node.l
                    : (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE) ? node.scrub : SuperstructurePreset.STOW_UPPER
                )
            )
        );
        // new EventTrigger("Align").onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(
            // superstructure.setPreset(node.l != SuperstructurePreset.MANUAL_OVERRIDE ? node.l : (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE ? node.scrub : SuperstructurePreset.STOW_UPPER))
        // )));
        SequentialCommandGroup place = new SequentialCommandGroup(
            superstructure.setPreset(node.l).until(() -> superstructure.atSetpoint()),
            new WaitCommand(/*node.l == SuperstructurePreset.L4 ? 1 :*/ 0.5),
            new ParallelDeadlineGroup(
                new WaitUntilCommand(superstructure.getCoralSensorIntake().negate().and(superstructure.getCoralSensorPlace().negate())),
                superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(node.l))
            )
        );
        SequentialCommandGroup scrub = new SequentialCommandGroup(
            superstructure.setPreset(node.scrub).until(() -> superstructure.atSetpoint()),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                scrubber.set(() -> Constants.Scrubber.MAX_ROTATIONS)
            )
        );

        if (!Utils.isSimulation()) {
            addCommands(new ParallelDeadlineGroup(move, scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations)));
            if (node.l != SuperstructurePreset.MANUAL_OVERRIDE) {
                addCommands(place);
            }
            if (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE) {
                addCommands(scrub);
            }
            addCommands(superstructure.setPreset(SuperstructurePreset.STOW_UPPER));
        } else {
            addCommands(AutoBuilder.pathfindThenFollowPath(path, constraints));
        }
    }
}
