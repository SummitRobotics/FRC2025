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
        ONE_BLUE(new Pose2d(2.678, 4.058, Rotation2d.fromDegrees(0))),
        TWO_BLUE(new Pose2d(3.565, 2.457, Rotation2d.fromDegrees(60))),
        THREE_BLUE(new Pose2d(5.414, 2.494, Rotation2d.fromDegrees(120))),
        FOUR_BLUE(new Pose2d(6.296, 4.032, Rotation2d.fromDegrees(180))),
        FIVE_BLUE(new Pose2d(5.378, 5.594, Rotation2d.fromDegrees(240))),
        SIX_BLUE(new Pose2d(3.625, 5.601, Rotation2d.fromDegrees(300))),
        ONE_RED(new Pose2d(8.593 + 6.296, 4.032, Rotation2d.fromDegrees(180))),
        TWO_RED(new Pose2d(8.593 + 5.378, 5.594, Rotation2d.fromDegrees(240))),
        THREE_RED(new Pose2d(8.593 + 3.625, 5.601, Rotation2d.fromDegrees(300))),
        FOUR_RED(new Pose2d(8.593 + 2.678, 4.058, Rotation2d.fromDegrees(0))),
        FIVE_RED(new Pose2d(8.593 + 3.565, 2.457, Rotation2d.fromDegrees(60))),
        SIX_RED(new Pose2d(8.593 + 5.414, 2.494, Rotation2d.fromDegrees(120)));

        public Pose2d waypoint;
        private HexSide(Pose2d waypoint) {
            this.waypoint = waypoint;
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

    public class Node {
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
        PathPlannerPath path = new PathPlannerPath(null, constraints, null, null);
        String pathName = "";
        // Name format is [side number][L/R] (e.g. 4R)
        pathName += node.side.name;
        pathName += node.l.name;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }
        addCommands(
            AutoBuilder.pathfindToPose(node.hexSide.waypoint, constraints, 0),
            drivetrain.applyRequest(() -> turnRequest).until(() -> drivetrain.getState().Pose.getRotation().minus(node.hexSide.waypoint.getRotation()).getDegrees() < 5),
            AutoBuilder.followPath(path),
            superstructure.setPreset(node.l).until(() -> superstructure.atSetpoint()),
            new ParallelRaceGroup(
                superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(node.l)),
                new WaitCommand(1)
            ),
            superstructure.setPreset(SuperstructurePreset.STOW)
        );
    }
}
