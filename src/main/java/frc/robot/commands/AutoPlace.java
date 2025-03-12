package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    // Ghost pointer positions to show which node is selected
    public static final Pose2d
        ONE_LEFT = new Pose2d(3.186 /*+ 0.01905*/, 4.194, Rotation2d.fromDegrees(0)),
        ONE_RIGHT = new Pose2d(3.186 /*+ 0.01905*/, 3.866, Rotation2d.fromDegrees(0)),
        TWO_LEFT = new Pose2d(3.693 /*+ 0.01905 * Math.cos(Math.PI / 3)*/, 2.982 /*+ 0.01905 * Math.sin(Math.PI / 3)*/, Rotation2d.fromDegrees(60)),
        TWO_RIGHT = new Pose2d(3.982 /*+ 0.01905 * Math.cos(Math.PI / 3)*/, 2.816 /*+ 0.01905 * Math.sin(Math.PI / 3)*/, Rotation2d.fromDegrees(60)),
        THREE_LEFT = new Pose2d(4.989 /*+ 0.01905 * Math.cos(2 * Math.PI / 3)*/, 2.818 /*+ 0.01905 * Math.sin(2 * Math.PI / 3)*/, Rotation2d.fromDegrees(120)),
        THREE_RIGHT = new Pose2d(5.283 /*+ 0.01905 * Math.cos(2 * Math.PI / 3)*/, 2.986 /*+ 0.01905 * Math.sin(2 * Math.PI / 3)*/, Rotation2d.fromDegrees(120)),
        FOUR_LEFT = new Pose2d(5.783 /*+ 0.01905 * Math.cos(Math.PI)*/, 3.863 /*+ 0.01905 * Math.sin(Math.PI)*/, Rotation2d.fromDegrees(180)),
        FOUR_RIGHT = new Pose2d(5.781 /*+ 0.01905 * Math.cos(Math.PI)*/, 4.188 /*+ 0.01905 * Math.sin(Math.PI)*/, Rotation2d.fromDegrees(180)),
        FIVE_LEFT = new Pose2d(5.264 /*+ 0.01905 * Math.cos(4 * Math.PI / 3)*/, 5.076 /*+ 0.01905 * Math.sin(4 * Math.PI / 3)*/, Rotation2d.fromDegrees(240)),
        FIVE_RIGHT = new Pose2d(4.992 /*+ 0.01905 * Math.cos(4 * Math.PI / 3)*/, 5.235 /*+ 0.01905 * Math.sin(4 * Math.PI / 3)*/, Rotation2d.fromDegrees(240)),
        SIX_LEFT = new Pose2d(3.973 /*+ 0.01905 * Math.cos(5 * Math.PI / 3)*/, 5.231 /*+ 0.01905 * Math.sin(5 * Math.PI / 3)*/, Rotation2d.fromDegrees(300)),
        SIX_RIGHT = new Pose2d(3.697 /*+ 0.01905 * Math.cos(5 * Math.PI / 3)*/, 5.061 /*+ 0.01905 * Math.sin(5 * Math.PI / 3)*/, Rotation2d.fromDegrees(300));

    public enum HexSide {
        ONE("1", ONE_LEFT, ONE_RIGHT),
        TWO("2", TWO_LEFT, TWO_RIGHT),
        THREE("3", THREE_LEFT, THREE_RIGHT),
        FOUR("4", FOUR_LEFT, FOUR_RIGHT),
        FIVE("5", FIVE_LEFT, FIVE_RIGHT),
        SIX("6", SIX_LEFT, SIX_RIGHT);

        public Pose2d leftPlace, rightPlace;
        public String name;
        private HexSide(String name, Pose2d leftPlace, Pose2d rightPlace) {
            this.name = name;
            this.rightPlace = rightPlace;
            this.leftPlace = leftPlace;
        }

        public Pose2d getPlacePose(Side side) {
            return side == Side.LEFT ? leftPlace : rightPlace;
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
            return "Hex: " + hexSide.name + ", Side: " + side.name + ", L: " + l.description + ", Scrub: " + scrub.description;
        }
    }

    public static SuperstructurePreset getScrubPose(HexSide hexSide) {
        return switch(hexSide) {
            case ONE -> SuperstructurePreset.L3_SCRUB;
            case TWO -> SuperstructurePreset.STOW_UPPER;
            case THREE -> SuperstructurePreset.L3_SCRUB;
            case FOUR -> SuperstructurePreset.STOW_UPPER;
            case FIVE -> SuperstructurePreset.L3_SCRUB;
            case SIX -> SuperstructurePreset.STOW_UPPER;
        };
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraintsSlow = new PathConstraints(
            1, 1.5,
            Units.degreesToRadians(270), Units.degreesToRadians(360));
    private PathConstraints constraintsFast = new PathConstraints(
        4, 3.5,
        360, 360
    );

    public AutoPlace(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, Node node, String suppliedPathName, boolean manipulatorSafe, boolean fast) {
        PathPlannerPath path;
        String pathName = "";
        // Name format is [side number][L/R] (e.g. 4R)
        pathName += node.hexSide.name;
        pathName += node.side.name;
        // If the node is L1, append 1 to the path name
        if (node.l == SuperstructurePreset.L1) pathName += "1";
        try {
            path = PathPlannerPath.fromPathFile(suppliedPathName.isEmpty() ? pathName : suppliedPathName);
        } catch (Exception e) {
            e.printStackTrace();
            throw (new RuntimeException("Loaded a path that does not exist."));
        }

        // Command to move the robot to the desired position along a path, running until path is complete
        Command move = new ParallelDeadlineGroup(
            // On the fly pathfinding to station, or follow the path if supplied
            new ConditionalCommand(
                // AutoBuilder.pathfindToPoseFlipped(node.hexSide.getPlacePose(node.side), constraintsSlow),
                // new PIDtoPose(drivetrain, node.hexSide.getPlacePose(node.side)),
                AutoBuilder.pathfindThenFollowPath(path, fast ? constraintsFast : constraintsSlow),
                // new ConditionalCommand(
                    // AutoBuilder.pathfindThenFollowPath(path, constraintsSlow),
                AutoBuilder.followPath(path),
                    // () -> onTheFly
                // ),
                () -> suppliedPathName.isEmpty()
            ),
            // Move the superstructure into a safe position for moving
            new SequentialCommandGroup(
                new ConditionalCommand(
                    superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE).withTimeout(1),
                    new InstantCommand(() -> {}),
                    () -> manipulatorSafe
                ),
                new ConditionalCommand(
                    // If going to L1, L2, or L3, set the superstructure to the desired position
                    new ConditionalCommand(
                        superstructure.setPresetWithAutoCenter(node.l),
                        superstructure.setPresetWithAutoCenter(SuperstructurePreset.STOW_UPPER),
                        () -> node.l != SuperstructurePreset.MANUAL_OVERRIDE
                    ),
                    // If going to L4 then use L4 intermediate
                    superstructure.setPresetWithAutoCenter(SuperstructurePreset.L4_INTERMEDIATE),
                    () -> node.l != SuperstructurePreset.L4
                )
            )
        );

        // Command to move superstructure to the desired position and shoot the coral
        SequentialCommandGroup place = new SequentialCommandGroup(
            // Move superstructure until at desired position, with a timeout
            superstructure.setPreset(node.l)
                .until(superstructure::atSetpoint)
                .withTimeout(node.l == SuperstructurePreset.L4 ? 0.75 : 0.5),
            // Wait some time if going to L4 (to allow the wrist to achieve pose)
            new WaitCommand((node.l == SuperstructurePreset.L4) ? 0.4 : 0),
            // Shoot out the coral
            new ParallelDeadlineGroup(
                // Rum until the shoot sensors are cleared, or for a timeout if going to L1
                node.l == SuperstructurePreset.L1 ?
                    new WaitCommand(2) :
                    new WaitUntilCommand(superstructure.getCoralSensorIntake().negate().and(superstructure.getCoralSensorPlace().negate())),
                new ConditionalCommand(
                    // Not going to L1; drive the shooter
                    superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(node.l)),
                    // Going to L1; run the L1 shoot sequence
                    new SequentialCommandGroup(
                        // Ready the coral for shooting (pulling it half-way in) with a timeout
                        superstructure.setPresetWithFarSpit(SuperstructurePreset.L1).withTimeout(1),
                        // Drive the shooter
                        superstructure.setPreset(SuperstructurePreset.L1_GO)
                    ),
                    () -> node.l != SuperstructurePreset.L1
                )
            )
        );

        // Command to run the algea scrubber (t-rex goes rawr), this is run after placing the coral
        SequentialCommandGroup scrub = new SequentialCommandGroup(
            // Move the superstructure to the desired scrub position, wait until at setpoint or timeout
            superstructure.setPreset(node.scrub)
                .until(superstructure::atSetpoint)
                .withDeadline(new WaitCommand(0.5)),
            // Wait some time if going to L4 (to allow the superstructure to settle?)
            new WaitCommand(node.l == SuperstructurePreset.L4 ? 0.5 : 0),
            // Drive the scrubber arms up until timeout
            scrubber.set(() -> Constants.Scrubber.MAX_ROTATIONS).withDeadline(new WaitCommand(0.5))
        );

        if (!Utils.isSimulation()) {
            // Move the robot to desired position, stowing scrubber along the way
            addCommands(new ParallelDeadlineGroup(move, scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations)));
            if (node.l != SuperstructurePreset.MANUAL_OVERRIDE) {
                // Move superstructure to the desired position and shoot the coral
                addCommands(place);
            }
            if (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE) {
                // Scrub an algea
                addCommands(scrub);
            }
        } else {
            // Simplified place in simulation
            addCommands(
                // Move the robot to desired position
                new ConditionalCommand(
                    AutoBuilder.pathfindToPoseFlipped(node.hexSide.getPlacePose(node.side), constraintsSlow),
                    AutoBuilder.followPath(path),
                    () -> suppliedPathName.isEmpty()
                )
                // No place or scrub in simulation
            );
        }
    }
}
