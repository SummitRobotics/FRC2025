package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;

public class AutoPlace extends SequentialCommandGroup {

    public enum HexSide {
        ONE_BLUE(new Pose2d(2.678, 4.058, Rotation2d.fromDegrees(0)), "1"),
        TWO_BLUE(new Pose2d(3.565, 2.457, Rotation2d.fromDegrees(60)), "2"),
        THREE_BLUE(new Pose2d(5.414, 2.494, Rotation2d.fromDegrees(120)), "3"),
        FOUR_BLUE(new Pose2d(6.296, 4.032, Rotation2d.fromDegrees(180)), "4"),
        FIVE_BLUE(new Pose2d(5.378, 5.594, Rotation2d.fromDegrees(240)), "5"),
        SIX_BLUE(new Pose2d(3.625, 5.601, Rotation2d.fromDegrees(300)), "6"),
        ONE_RED(new Pose2d(8.593 + 6.296, 4.032, Rotation2d.fromDegrees(180)), "1"),
        TWO_RED(new Pose2d(8.593 + 5.378, 5.594, Rotation2d.fromDegrees(240)), "2"),
        THREE_RED(new Pose2d(8.593 + 3.625, 5.601, Rotation2d.fromDegrees(300)), "3"),
        FOUR_RED(new Pose2d(8.593 + 2.678, 4.058, Rotation2d.fromDegrees(0)), "4"),
        FIVE_RED(new Pose2d(8.593 + 3.565, 2.457, Rotation2d.fromDegrees(60)), "5"),
        SIX_RED(new Pose2d(8.593 + 5.414, 2.494, Rotation2d.fromDegrees(120)), "6");

        public Pose2d waypoint;
        public String name;
        private HexSide(Pose2d waypoint, String name) {
            this.waypoint = waypoint;
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
        public Node(SuperstructurePreset l, HexSide hexSide, Side side) {
            this.l = l;
            this.side = side;
            this.hexSide = hexSide;
        }
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));
    // To turn the robot correctly between paths
    private SwerveRequest.FieldCentricFacingAngle turnRequest = new SwerveRequest.FieldCentricFacingAngle();

    public AutoPlace(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Node node) {
        turnRequest.TargetDirection = node.hexSide.waypoint.getRotation();
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
        addCommands(
            AutoBuilder.pathfindToPose(node.hexSide.waypoint, constraints, 0),
            // drivetrain.applyRequest(() -> turnRequest).until(() -> drivetrain.getState().Pose.getRotation().minus(node.hexSide.waypoint.getRotation()).getDegrees() < 5),
            AutoBuilder.followPath(path),
            superstructure.setPreset(node.l).until(() -> superstructure.atSetpoint()),
            new ParallelRaceGroup(
                superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(node.l)),
                new WaitCommand(1)
            ),
            superstructure.setPreset(SuperstructurePreset.STOW)
        );
        // addRequirements(drivetrain, superstructure); // Is this necessary?
    }
}
