package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
            return "Hex: " + hexSide.name + ", Side: " + side.name + ", L: " + l.description + ", Scrub: " + scrub.description;
        }
    }

    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));

    public AutoPlace(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, Node node) {
        this(drivetrain, superstructure, scrubber, node, "");
    }

    public AutoPlace(CommandSwerveDrivetrain drivetrain, Superstructure superstructure, Scrubber scrubber, Node node, String suppliedPathName) {
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
            // new InstantCommand(() -> drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())).repeatedly().withDeadline(new WaitCommand(0.3)),
            // new ParallelDeadlineGroup(
                // AutoBuilder.pathfindToPoseFlipped(path.getStartingHolonomicPose().get(), constraints),
                // new ConditionalCommand(
                    // superstructure.setPreset(node.l),
                    // new ConditionalCommand(
                        // superstructure.setPreset(SuperstructurePreset.L4_INTERMEDIATE),
                        // new InstantCommand(() -> {}),
                        // () -> node.l == SuperstructurePreset.L4 || node.l == SuperstructurePreset.L3
                    // ),
                    // () -> node.l == SuperstructurePreset.L2
                // )
            // ),
            // new ParallelDeadlineGroup(
                // AutoBuilder.followPath(path),
                // superstructure.setPreset(
                    // node.l != SuperstructurePreset.MANUAL_OVERRIDE ? node.l
                    // : (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE) ? node.scrub : SuperstructurePreset.STOW_UPPER
                // )
            // )

            // On the fly pathfinding to station, or follow the path if supplied
            new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(path, constraints),
                AutoBuilder.followPath(path),
                () -> suppliedPathName.isEmpty()
            ),
            // Move the superstructure into a safe position for moving
            new ConditionalCommand(
                // If going to L1 or L2, set the superstructure to the desired position
                superstructure.setPreset(node.l),
                new ConditionalCommand(
                    // If going to L3 or L4 then use L4 intermediate
                    superstructure.setPreset(SuperstructurePreset.L4_INTERMEDIATE),
                    // Otherwise manual or other unhandled condition
                    new InstantCommand(() -> {}),
                    () -> node.l == SuperstructurePreset.L3 || node.l == SuperstructurePreset.L4
                ),
                () -> node.l == SuperstructurePreset.L1 || node.l == SuperstructurePreset.L2
            )
        );
        // new EventTrigger("Align").onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(
            // superstructure.setPreset(node.l != SuperstructurePreset.MANUAL_OVERRIDE ? node.l : (node.scrub != SuperstructurePreset.MANUAL_OVERRIDE ? node.scrub : SuperstructurePreset.STOW_UPPER))
        // )));

        // Command to move superstructure to the desired position and shoot the coral
        SequentialCommandGroup place = new SequentialCommandGroup(
            // Move superstructure until at desired position, with a timeout
            superstructure.setPreset(node.l)
                .until(superstructure::atSetpoint)
                .withTimeout(node.l == SuperstructurePreset.L4 ? 0.75 : 0.5),
            // Wait some time if going to L3/L4 (to allow the wrist to achieve pose)
            new WaitCommand((node.l == SuperstructurePreset.L3) || (node.l == SuperstructurePreset.L4) ? 0.5 : 0),
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

            // addCommands(superstructure.setPreset(SuperstructurePreset.STOW_UPPER).until(superstructure::atSetpoint).withTimeout(0.5));

            // Back up slightly while stowing the superstructure for a period of time
            Timer timer = new Timer();
            addCommands(
                new ParallelCommandGroup(
                    // Move the superstructure to the desired stow position
                    superstructure.setPreset(SuperstructurePreset.STOW_UPPER),
                    // Adjust target speed to accelerate backwards (-X in robot centric)
                    new SequentialCommandGroup(
                        new InstantCommand(timer::restart),
                        new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-timer.get() * 4))).repeatedly()
                    )
                ).withDeadline(new WaitCommand(0.5))
            );
        } else {
            // Simplified place in simulation
            Timer timer = new Timer();
            addCommands(
                // Move the robot to desired position, stowing scrubber along the way
                new ConditionalCommand(
                    AutoBuilder.pathfindThenFollowPath(path, constraints),
                    AutoBuilder.followPath(path),
                    () -> suppliedPathName.isEmpty()
                ),

                /*
                 * No place or scrub in simulation
                 */

                // Adjust target speed to accelerate backwards (-X in robot centric) for a period of time
                new InstantCommand(timer::restart),
                new InstantCommand(() -> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-timer.get() * 4))).repeatedly().withDeadline(new WaitCommand(0.5))
            );
        }
    }
}
