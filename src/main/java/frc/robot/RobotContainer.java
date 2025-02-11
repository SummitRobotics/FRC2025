// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPlace.HexSide;
import frc.robot.commands.AutoPlace.Node;
import frc.robot.commands.AutoPlace.Side;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.ButtonBox.Button;
import frc.robot.oi.CommandControllerWrapper;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scrubber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.Functions;
import frc.robot.utilities.lists.Constants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.825).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandControllerWrapper driverController;
    private final CommandControllerWrapper gunnerController;
    private final ButtonBox buttonBox = new ButtonBox(Constants.OI.BUTTON_BOX);

    // Subsystems
    @Logged(name = "Drivetrain")
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    @Logged(name = "Superstructure")
    public final Superstructure superstructure = new Superstructure(buttonBox);
    public final Scrubber scrubber = new Scrubber(superstructure::pivotRotations);
    
    // Path follower
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<SuperstructurePreset> lChooser;
    private final SendableChooser<HexSide> hexSideChooser;
    private final SendableChooser<Side> leftRightChooser;
    private final SendableChooser<SuperstructurePreset> scrubChooser;

    // Button-based node chooser
    // private Side leftRight = Side.LEFT;
    // private SuperstructurePreset l = SuperstructurePreset.L1;
    // private HexSide hexSide = HexSide.ONE;
    // private Command generateSelectCommand(HexSide hexSide, Side leftRight) {
        // return new InstantCommand(() -> {
            // this.hexSide = hexSide;
            // this.leftRight = leftRight;
            // driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(l, hexSide, leftRight)));
        // });
    // }
    // private final Command generateSelectCommand(SuperstructurePreset l) {
        // return new InstantCommand(() -> {
            // this.l = l;
            // driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(l, hexSide, leftRight)));
        // });
    // }

    // private final Command
        // selectOneLeft = generateSelectCommand(HexSide.ONE, Side.LEFT),
        // selectOneRight = generateSelectCommand(HexSide.ONE, Side.RIGHT),
        // selectTwoLeft = generateSelectCommand(HexSide.TWO, Side.LEFT),
        // selectTwoRight = generateSelectCommand(HexSide.TWO, Side.RIGHT),
        // selectThreeLeft = generateSelectCommand(HexSide.THREE, Side.LEFT),
        // selectThreeRight = generateSelectCommand(HexSide.THREE, Side.RIGHT),
        // selectFourLeft = generateSelectCommand(HexSide.FOUR, Side.LEFT),
        // selectFourRight = generateSelectCommand(HexSide.FOUR, Side.RIGHT),
        // selectFiveLeft = generateSelectCommand(HexSide.FIVE, Side.LEFT),
        // selectFiveRight = generateSelectCommand(HexSide.FIVE, Side.RIGHT),
        // selectSixLeft = generateSelectCommand(HexSide.SIX, Side.LEFT),
        // selectSixRight = generateSelectCommand(HexSide.SIX, Side.RIGHT),
        // selectL1 = generateSelectCommand(SuperstructurePreset.L1),
        // selectL2 = generateSelectCommand(SuperstructurePreset.L2),
        // selectL3 = generateSelectCommand(SuperstructurePreset.L3),
        // selectL4 = generateSelectCommand(SuperstructurePreset.L4);

    // Field2d object for simulation
    private final Field2d field = new Field2d();

    // Ghost pointer positions to show which node is selected
    private final Pose2d
        ONE_LEFT = new Pose2d(3.186, 4.194, Rotation2d.fromDegrees(0)),
        ONE_RIGHT = new Pose2d(3.187, 3.866, Rotation2d.fromDegrees(0)),
        TWO_LEFT = new Pose2d(3.693, 2.982, Rotation2d.fromDegrees(60)),
        TWO_RIGHT = new Pose2d(3.982, 2.816, Rotation2d.fromDegrees(60)),
        THREE_LEFT = new Pose2d(4.989, 2.818, Rotation2d.fromDegrees(120)),
        THREE_RIGHT = new Pose2d(5.283, 2.986, Rotation2d.fromDegrees(120)),
        FOUR_LEFT = new Pose2d(5.783, 3.863, Rotation2d.fromDegrees(180)),
        FOUR_RIGHT = new Pose2d(5.781, 4.188, Rotation2d.fromDegrees(180)),
        FIVE_LEFT = new Pose2d(5.264, 5.076, Rotation2d.fromDegrees(240)),
        FIVE_RIGHT = new Pose2d(4.992, 5.235, Rotation2d.fromDegrees(240)),
        SIX_LEFT = new Pose2d(3.973, 5.231, Rotation2d.fromDegrees(300)),
        SIX_RIGHT = new Pose2d(3.697, 5.061, Rotation2d.fromDegrees(300));

    public RobotContainer() {
        // Check if PS5 controllers should be used
        boolean usePS5Controllers = Boolean.parseBoolean(System.getenv("USE_PS5_CONTROLLERS"));

        if (usePS5Controllers) {
            driverController = new CommandControllerWrapper(new CommandPS5Controller(Constants.OI.DRIVER_PS5));
            gunnerController = new CommandControllerWrapper(new CommandPS5Controller(Constants.OI.GUNNER_PS5));
            System.out.println("Controller Type: PS5");            
        } else {
            driverController = new CommandControllerWrapper(new CommandXboxController(Constants.OI.DRIVER_XBOX));
            gunnerController = new CommandControllerWrapper(new CommandXboxController(Constants.OI.GUNNER_XBOX));
            System.out.println("Controller Type: Xbox");            
        }

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        lChooser = new SendableChooser<SuperstructurePreset>();
        hexSideChooser = new SendableChooser<HexSide>();
        leftRightChooser = new SendableChooser<Side>();
        scrubChooser = new SendableChooser<SuperstructurePreset>();
        lChooser.setDefaultOption("None", SuperstructurePreset.MANUAL_OVERRIDE); // TODO - using MANUAL_OVERRIDE for the flag is sloppy
        lChooser.addOption("L1", SuperstructurePreset.L1);
        lChooser.addOption("L2", SuperstructurePreset.L2);
        lChooser.addOption("L3", SuperstructurePreset.L3);
        lChooser.addOption("L4", SuperstructurePreset.L4);
        hexSideChooser.setDefaultOption("1", AutoPlace.HexSide.ONE);
        hexSideChooser.addOption("2", AutoPlace.HexSide.TWO);
        hexSideChooser.addOption("3", AutoPlace.HexSide.THREE);
        hexSideChooser.addOption("4", AutoPlace.HexSide.FOUR);
        hexSideChooser.addOption("5", AutoPlace.HexSide.FIVE);
        hexSideChooser.addOption("6", AutoPlace.HexSide.SIX);
        leftRightChooser.setDefaultOption("Left", AutoPlace.Side.LEFT);
        leftRightChooser.addOption("Right", AutoPlace.Side.RIGHT);
        scrubChooser.setDefaultOption("None", SuperstructurePreset.MANUAL_OVERRIDE);
        scrubChooser.addOption("L2", SuperstructurePreset.STOW_UPPER);
        scrubChooser.addOption("L3", SuperstructurePreset.L3_SCRUB);
        // These weren't changing the bound command properly before this got added, so it seems like the rebinds are necessary.
        lChooser.onChange((SuperstructurePreset l) -> {
            driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, scrubber, new Node(l, hexSideChooser.getSelected(), leftRightChooser.getSelected(), scrubChooser.getSelected())));
        });
        hexSideChooser.onChange((HexSide hexSide) -> {
            driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, scrubber, new Node(lChooser.getSelected(), hexSide, leftRightChooser.getSelected(), scrubChooser.getSelected())));
        });
        leftRightChooser.onChange((Side leftRight) -> {
            driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, scrubber, new Node(lChooser.getSelected(), hexSideChooser.getSelected(), leftRight, scrubChooser.getSelected())));
        });
        scrubChooser.onChange((SuperstructurePreset scrub) -> {
            driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, scrubber, new Node(lChooser.getSelected(), hexSideChooser.getSelected(), leftRightChooser.getSelected(), scrub)));
        });
        SmartDashboard.putData("L Chooser", lChooser);
        SmartDashboard.putData("Hex Side Chooser", hexSideChooser);
        SmartDashboard.putData("Left-Right Chooser", leftRightChooser);
        SmartDashboard.putData("Scrub Chooser", scrubChooser);
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Superstructure", superstructure);
        // SmartDashboard.putData("One Left", selectOneLeft);
        // SmartDashboard.putData("One Right", selectOneRight);
        // SmartDashboard.putData("Two Left", selectTwoLeft);
        // SmartDashboard.putData("Two Right", selectTwoRight);
        // SmartDashboard.putData("Three Left", selectThreeLeft);
        // SmartDashboard.putData("Three Right", selectThreeRight);
        // SmartDashboard.putData("Four Left", selectFourLeft);
        // SmartDashboard.putData("Four Right", selectFourRight);
        // SmartDashboard.putData("Five Left", selectFiveLeft);
        // SmartDashboard.putData("Five Right", selectFiveRight);
        // SmartDashboard.putData("Six Left", selectSixLeft);
        // SmartDashboard.putData("Six Right", selectSixRight);
        // SmartDashboard.putData("L1", selectL1);
        // SmartDashboard.putData("L2", selectL2);
        // SmartDashboard.putData("L3", selectL3);
        // SmartDashboard.putData("L4", selectL4);
        SmartDashboard.putData("Field", field);
        configureBindings();
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        driverController.back().and(driverController.y()).whileTrue(superstructure.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(superstructure.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(superstructure.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(superstructure.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Superstructure MOs
        buttonBox.getTrigger(Button.MO_PRESET).whileTrue(superstructure.setManual(
            () -> gunnerController.getLeftTriggerAxis() * Constants.Elevator.MAX_ROTATIONS,
            () -> Constants.Manipulator.CLEAR_OF_ELEVATOR_ROTATIONS + gunnerController.getRightTriggerAxis() * (Constants.Manipulator.MAX_ROTATIONS - Constants.Manipulator.CLEAR_OF_ELEVATOR_ROTATIONS),
            () -> (gunnerController.leftBumper().getAsBoolean() ? -1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? 1 : 0),
            () -> (gunnerController.leftBumper().getAsBoolean() ? 1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? -1 : 0)
        ));
        // Scrubber MOs
        buttonBox.getTrigger(Button.MO_PRESET).whileTrue(scrubber.set(() -> gunnerController.x().getAsBoolean() ? Constants.Scrubber.MAX_ROTATIONS : Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations));

        // Bind the button box presets
        for (SuperstructurePreset preset : SuperstructurePreset.values()) {
            if (preset.button != null) buttonBox.getTrigger(preset.button).onTrue(
                new SequentialCommandGroup(
                    scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations).until(scrubber::safe),
                    superstructure.setPresetWithBeltOverride(
                        preset,
                        () -> (gunnerController.leftBumper().getAsBoolean() ? -1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? 1 : 0),
                        () -> (gunnerController.leftBumper().getAsBoolean() ? 1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? -1 : 0)
            )));
        }
        buttonBox.getTrigger(Button.GO_PRESET).onTrue(superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(superstructure.getState())));

        drivetrain.registerTelemetry(logger::telemeterize);
        driverController.a().whileTrue(new AutoPlace(drivetrain, superstructure, scrubber, new Node(lChooser.getSelected(), hexSideChooser.getSelected(), leftRightChooser.getSelected(), scrubChooser.getSelected())));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void robotPeriodic() {
        buttonBox.sendMessage();
        // Update Field2d object
        field.setRobotPose(drivetrain.getState().Pose);
        boolean left = leftRightChooser.getSelected() == Side.LEFT;
        Pose2d ghostPose = switch (hexSideChooser.getSelected()) {
            case ONE -> left ? ONE_LEFT : ONE_RIGHT;
            case TWO -> left ? TWO_LEFT : TWO_RIGHT;
            case THREE -> left ? THREE_LEFT : THREE_RIGHT;
            case FOUR -> left ? FOUR_LEFT : FOUR_RIGHT;
            case FIVE -> left ? FIVE_LEFT : FIVE_RIGHT;
            case SIX -> left ? SIX_LEFT : SIX_RIGHT;
        };
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            ghostPose = Functions.mirrorPoseToRed(ghostPose);
        }
        field.getObject("PathTarget").setPose(ghostPose);

        // Get the active path from the network tables and set it onto the field object
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/PathPlanner");
        NetworkTableEntry activePathEntry = table.getEntry("activePath");

        byte[] activePathBytes = activePathEntry.getRaw(new byte[0]);

        // Deserialize the raw bytes into Pose2d objects
        List<Pose2d> activePath = new ArrayList<>();
        ByteBuffer buffer = ByteBuffer.wrap(activePathBytes).order(ByteOrder.LITTLE_ENDIAN);
        while (buffer.remaining() >= 24) {
            double x = buffer.getDouble();
            double y = buffer.getDouble();
            double theta = buffer.getDouble();
            activePath.add(new Pose2d(new Translation2d(x, y), new Rotation2d(theta)));
        }

        if (!activePath.isEmpty()) {
            // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(activePath, new TrajectoryConfig(2.0, 2.0));
            // field.getObject("PathTrajectory").setTrajectory(trajectory);
            field.getObject("PathTrajectory").setPoses(activePath);
        }
    }
}
