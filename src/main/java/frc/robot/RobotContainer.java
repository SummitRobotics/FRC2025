// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignRequestToF;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPickup.AutoSegment;
import frc.robot.commands.AutoPickup.CoralStationSide;
import frc.robot.commands.AutoPlace.HexSide;
import frc.robot.commands.AutoPlace.Node;
import frc.robot.commands.AutoPlace.Side;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.ButtonBox.Button;
import frc.robot.oi.CommandControllerWrapper;
import frc.robot.oi.JoystickResponseCurve;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Scrubber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.Functions;
import frc.robot.utilities.PathProvider;
import frc.robot.utilities.lists.Constants;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.PriorityQueue;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.825).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // #region driver_tuning
    // Driver controller tuning enabled when Constants.ENABLE_DRIVER_CONTROLLER_TUNING is true
    private double translationDeadband = 0.05; // 5% deadband
    private double rotationDeadband = 0.05; // 5% deadband
    private double curveScale = 2.0; // Shared between translation and rotation curves, used with exponential curve
    private double dualRateBreakpoint = 0.5; // Shared between translation and rotation curves, used with dual rate curve

    private double driveRequestTransDeadband = 0.0;
    private double driveRequestRotDeadband = 0.0;

    // Open-loop voltage and LINEAR curves to match current behavior
    private DriveRequestType currentDriveRequestType = DriveRequestType.OpenLoopVoltage;
    private JoystickResponseCurve.CurveType currentDriveCurveType = JoystickResponseCurve.CurveType.LINEAR;
    private JoystickResponseCurve.CurveType currentRotationCurveType = JoystickResponseCurve.CurveType.MILD;

    private final String DRIVER_TUNING_BASE_PATH = "DriverTuning/";
    private final SendableChooser<DriveRequestType> driveRequestTypeChooser = new SendableChooser<>();
    private final SendableChooser<JoystickResponseCurve.CurveType> driveCurveChooser = new SendableChooser<>();
    private final SendableChooser<JoystickResponseCurve.CurveType> rotationCurveChooser = new SendableChooser<>();
    // #endregion

    /* Setting up bindings for necessary control of the swerve drive platform */
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * driveRequestTransDeadband)
            .withRotationalDeadband(MaxAngularRate * driveRequestTransDeadband)
            .withDriveRequestType(currentDriveRequestType);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandControllerWrapper driverController;
    private final CommandControllerWrapper gunnerController;
    private final ButtonBox buttonBox = new ButtonBox(Constants.OI.BUTTON_BOX);

    // Response curves for joystick inputs
    private JoystickResponseCurve driveCurve =
        new JoystickResponseCurve(JoystickResponseCurve.CurveType.LINEAR, translationDeadband);
    private JoystickResponseCurve rotationCurve =
        new JoystickResponseCurve(JoystickResponseCurve.CurveType.LINEAR, rotationDeadband);

    // Subsystems
    // @Logged(name = "Drivetrain")
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // @Logged(name = "Superstructure")
    public final Superstructure superstructure = new Superstructure(buttonBox);
    // @Logged(name = "Scrubber")
    public final Scrubber scrubber = new Scrubber(superstructure::pivotRotations);
    public final LEDSubsystem ledSubsystem = new LEDSubsystem(Constants.LED.PWM_PORT, Constants.LED.LED_COUNT);
    // @Logged(name = "Climb")
    public final Climb climb = new Climb();

    // Auto-align chooser
    private final SendableChooser<Command> autoChooser;
    private final Command defaultAutoCommand = new InstantCommand(() -> {});
    private final SendableChooser<SuperstructurePreset> lChooser;
    private final SendableChooser<HexSide> hexSideChooser;
    private final SendableChooser<Side> leftRightChooser;
    private final SendableChooser<Boolean> scrubChooser;
    private final SendableChooser<Boolean> reefChoiceAssist;
    // private final SendableChooser<Boolean> backwardsChooser;
    private HexSide lastReef;
    private boolean lastBackwards;
    private Runnable rebindAutoPlace;

    // Auto
    private final SendableChooser<CoralStationSide> autoCoralStationChoice;
    private final SendableChooser<AutoSegment> autoSegmentChoice;
    private Timer flipUpTimer = new Timer();

    // Cycle timer variables
    private final Timer cycleTimer = new Timer();
    private int cycleCount = 0;
    private boolean timerRunning = false;
    private double lastCycleTime = 0.0;
    private double totalCycleTime = 0.0;
    private double averageCycleTime = 0.0;
    private double medianCycleTime = 0.0;
    private PriorityQueue<Double> medianMaxHeap = new PriorityQueue<>(Collections.reverseOrder()); // max heap for smaller values
    private PriorityQueue<Double> medianMinHeap = new PriorityQueue<>(); // min heap for larger values

    // Field2d object for simulation
    private final Field2d field = new Field2d();

    private String[] GetPathNames() {
        try {
            // Get the deploy/pathplanner/paths directory
            Path pathPlannerDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/paths");

            // List all files in the directory and collect their basenames
            return Files.list(pathPlannerDir)
                    .filter(Files::isRegularFile) // Only include regular files
                    .map(path -> path.getFileName().toString().replaceFirst("\\.path$", "")) // Remove ".path" extension
                    .toArray(String[]::new); // Convert to String array
        } catch (IOException e) {
            e.printStackTrace();
            System.err.println("Failed to load path names from deploy/pathplanner/paths");
            return new String[0]; // Return an empty array if an error occurs
        }
    }

    public RobotContainer() {
        // Preload our paths
        PathProvider.initialize(this.GetPathNames());

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
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Do Nothing", defaultAutoCommand);
        lChooser = new SendableChooser<SuperstructurePreset>();
        hexSideChooser = new SendableChooser<HexSide>();
        leftRightChooser = new SendableChooser<Side>();
        scrubChooser = new SendableChooser<Boolean>();
        reefChoiceAssist = new SendableChooser<Boolean>();
        // backwardsChooser = new SendableChooser<Boolean>();
        lChooser.addOption("L4", SuperstructurePreset.L4);
        lChooser.addOption("L3", SuperstructurePreset.L3);
        lChooser.addOption("L2", SuperstructurePreset.L2);
        lChooser.addOption("L1", SuperstructurePreset.L1);
        lChooser.setDefaultOption("None", SuperstructurePreset.MANUAL_OVERRIDE);
        hexSideChooser.setDefaultOption("1", AutoPlace.HexSide.ONE);
        hexSideChooser.addOption("2", AutoPlace.HexSide.TWO);
        hexSideChooser.addOption("3", AutoPlace.HexSide.THREE);
        hexSideChooser.addOption("4", AutoPlace.HexSide.FOUR);
        hexSideChooser.addOption("5", AutoPlace.HexSide.FIVE);
        hexSideChooser.addOption("6", AutoPlace.HexSide.SIX);
        leftRightChooser.setDefaultOption("A", AutoPlace.Side.LEFT);
        leftRightChooser.addOption("B", AutoPlace.Side.RIGHT);
        scrubChooser.setDefaultOption("No", false);
        scrubChooser.addOption("Yes", true);
        // Rebind upon scoring position selection change
        reefChoiceAssist.setDefaultOption("No", false);
        reefChoiceAssist.addOption("Yes", true);
        // backwardsChooser.setDefaultOption("No", false);
        // backwardsChooser.addOption("Yes", true);
        lastReef = hexSideChooser.getSelected();
        lastBackwards = false;
        rebindAutoPlace = () -> {
            driverController.leftBumper().whileTrue(new AutoPlace(
                drivetrain,
                superstructure,
                scrubber,
                new Node(
                    lChooser.getSelected(),
                    getHex(),
                    leftRightChooser.getSelected(),
                    (scrubChooser.getSelected() ? AutoPlace.getScrubPose(getHex()) : SuperstructurePreset.MANUAL_OVERRIDE)
                ), "", false, false, getBackwards())
                    .finallyDo(() -> {
                        // CommandScheduler.getInstance().schedule(
                        superstructure.setDefaultCommand(
                            superstructure.setPreset(lChooser.getSelected() == SuperstructurePreset.L1
                                ? SuperstructurePreset.STOW_LOWER : SuperstructurePreset.STOW_UPPER)
                        );
                    })
                );
        };
        lChooser.onChange((SuperstructurePreset l) -> rebindAutoPlace.run());
        leftRightChooser.onChange((Side leftRight) -> rebindAutoPlace.run());
        scrubChooser.onChange((Boolean doScrub) -> rebindAutoPlace.run());

        // Combinatoric auto-chooser thing
        autoCoralStationChoice = new SendableChooser<CoralStationSide>();
        autoSegmentChoice = new SendableChooser<AutoSegment>();
        autoCoralStationChoice.setDefaultOption("Left", CoralStationSide.LEFT);
        autoCoralStationChoice.addOption("Right", CoralStationSide.RIGHT);
        autoCoralStationChoice.addOption("None", CoralStationSide.NONE);
        autoSegmentChoice.setDefaultOption("Do First", AutoSegment.DO_FIRST);
        autoSegmentChoice.addOption("Do Second", AutoSegment.DO_SECOND);
        autoSegmentChoice.addOption("Do Third", AutoSegment.DO_THIRD);
        SmartDashboard.putData("L Chooser", lChooser);
        SmartDashboard.putData("Hex Side Chooser", hexSideChooser);
        SmartDashboard.putData("Left-Right Chooser", leftRightChooser);
        SmartDashboard.putData("Scrub Chooser", scrubChooser);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Reef Choice Assist", reefChoiceAssist);
        if (Constants.DEBUG_LOG_ENABLED) {
            SmartDashboard.putData("Superstructure", superstructure);
            SmartDashboard.putData("Drivetrain", drivetrain);
            SmartDashboard.putData("Climb", climb);
        }
        SmartDashboard.putData("Field", field);
        // Climb
        SmartDashboard.putData("Extend", climb.extend());
        SmartDashboard.putData("Lift", climb.lift().repeatedly().alongWith(superstructure.setPreset(SuperstructurePreset.CLIMB_STOW)));
        SmartDashboard.putData("Retract", climb.retract());
        // Instant scrub button
        SmartDashboard.putData("Scrub", new SequentialCommandGroup(
            // Move the superstructure to the desired scrub position, wait until at setpoint or timeout
            superstructure.setPreset(SuperstructurePreset.STOW_UPPER)
                .until(superstructure::atSetpoint)
                .withDeadline(new WaitCommand(0.5)),
            // Wait some time if going to L4 (to allow the superstructure to settle?)
            // Drive the scrubber arms up until timeout
            scrubber.set(() -> Constants.Scrubber.MAX_ROTATIONS).withDeadline(new WaitCommand(0.5))
        ));
        // Add further auto options (may be best moved to a separate file)
        try {
            autoChooser.addOption(
                "Left",
                new SequentialCommandGroup(
                    new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.FIVE, Side.RIGHT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftStart", false, true, false),
                    new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.LEFT, "ThreePieceLeftA", SuperstructurePreset.MANUAL_OVERRIDE, false),
                    new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.SIX, Side.LEFT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftB", true, true, false),
                    new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.LEFT, "ThreePieceLeftC", SuperstructurePreset.MANUAL_OVERRIDE, false),
                    new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.SIX, Side.RIGHT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftD", true, true, false),
                    new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.LEFT, "ThreePieceLeftE", AutoPlace.getScrubPose(HexSide.SIX), false)
                )
            );
            // autoChooser.addOption(
                // "Left L3",
                // new SequentialCommandGroup(
                    // new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.FIVE, Side.RIGHT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftStart", false, true, false),
                    // new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.LEFT, "ThreePieceLeftA", SuperstructurePreset.MANUAL_OVERRIDE, false),
                    // new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.SIX, Side.LEFT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftB", true, true, false),
                    // new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.LEFT, "ThreePieceLeftC", AutoPlace.getScrubPose(HexSide.SIX), false),
                    // new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L3, HexSide.SIX, Side.RIGHT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftD", true, true, false),
                    // new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.LEFT, "ThreePieceLeftE", SuperstructurePreset.MANUAL_OVERRIDE, true),
                    // new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L3, HexSide.SIX, Side.LEFT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceLeftF", true, true, false)
                // )
            // );
            autoChooser.addOption(
                "Center",
                new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.FOUR, Side.LEFT, SuperstructurePreset.STOW_UPPER), "", false, true, false)
            );
            autoChooser.addOption(
                "Right",
                new SequentialCommandGroup(
                    new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.THREE, Side.LEFT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceRightStart", false, true, false),
                    new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.RIGHT, "ThreePieceRightA", SuperstructurePreset.MANUAL_OVERRIDE, false),
                    new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.TWO, Side.RIGHT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceRightB", true, true, false),
                    new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.RIGHT, "ThreePieceRightC", SuperstructurePreset.MANUAL_OVERRIDE, false),
                    new AutoPlace(drivetrain, superstructure, scrubber, new Node(SuperstructurePreset.L4, HexSide.TWO, Side.LEFT, SuperstructurePreset.MANUAL_OVERRIDE), "ThreePieceRightD", true, true, false),
                    new AutoPickup(drivetrain, superstructure, scrubber, () -> CoralStationSide.RIGHT, "ThreePieceRightE", AutoPlace.getScrubPose(HexSide.TWO), false)
                )
            );
        } catch (Exception e) {
            throw new RuntimeException();
        }
        configureBindings();
        CameraServer.startAutomaticCapture();
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
        initializeDashboardTuning();
    }

    private void updateDashboard()
    {
        // Fetch info from the DriverStation and update the FMSInfo table
        double matchTime = DriverStation.getMatchTime();
        NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("MatchTime").setDouble(matchTime);

        SmartDashboard.putNumber("Current Cycle Time", Math.round(cycleTimer.get() * 10.0) / 10.0);
        SmartDashboard.putNumber("Last Cycle Time", Math.round(lastCycleTime * 10.0) / 10.0);
        SmartDashboard.putNumber("Cycle Count", cycleCount);
        SmartDashboard.putNumber("Average Cycle Time", Math.round(averageCycleTime * 10.0) / 10.0);
        SmartDashboard.putNumber("Median Cycle Time", Math.round(medianCycleTime * 10.0) / 10.0);

        // Check if a non-default auto command is selected
        boolean isAutoSelected = autoChooser.getSelected() != defaultAutoCommand;
        SmartDashboard.putBoolean("Auto Selected", isAutoSelected);
    }

    // Track whether the robot has left the RECEIVE state
    private boolean hasLeftReceiveState = true;

    /**
     * Configure cycle timer triggers for tracking cycle times during a match
     */
    private void configureCycleTimerTriggers() {
        // Stop cycle timer when robot is disabled
        new Trigger(() -> DriverStation.isDisabled())
            .onTrue(new InstantCommand(() -> {
                if (timerRunning) {
                    // Just stop the timer without counting it as a cycle
                    cycleTimer.stop();
                    timerRunning = false;
                }
            }));

        // Start cycle timer at match start if coral is pre-loaded
        new Trigger(() -> DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled())
            .onTrue(new InstantCommand(() -> {
                // Only trigger once at the beginning of a match when we have a pre-loaded piece
                if (!timerRunning && superstructure.getCoralSensorIntake().getAsBoolean() &&
                    superstructure.getCoralSensorPlace().getAsBoolean()) {
                    System.out.println("Match started with pre-loaded coral - starting cycle timer");
                    cycleTimer.reset();
                    cycleTimer.start();
                    timerRunning = true;
                }
            }));

        // Continue with regular cycle timer logic for subsequent pieces
        new Trigger(() -> superstructure.getState() == SuperstructurePreset.RECEIVE)
            .and(superstructure.getCoralSensorIntake())
            .and(superstructure.getCoralSensorPlace())
            .onTrue(new InstantCommand(() -> {
                // Only trigger if we've left and re-entered the RECEIVE state.
                if (hasLeftReceiveState) {
                    hasLeftReceiveState = false;

                    if (!timerRunning) {
                        // First cycle or new cycle after scoring
                        cycleTimer.reset();
                        cycleTimer.start();
                        timerRunning = true;
                    } else {
                        // Complete a cycle
                        lastCycleTime = cycleTimer.get();
                        cycleCount++;

                        // Update statistics
                        totalCycleTime += lastCycleTime;
                        averageCycleTime = totalCycleTime / cycleCount;
                        addToMedianCalculation(lastCycleTime);

                        cycleTimer.reset(); // Reset for next cycle
                    }
                }
            }));

        // Reset the flag when leaving the RECEIVE state
        new Trigger(() -> superstructure.getState() != SuperstructurePreset.RECEIVE)
            .onTrue(new InstantCommand(() -> {
                hasLeftReceiveState = true;
            }));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Get joystick inputs
                double leftY = -driverController.getLeftY(); // Drive forward with negative Y (Joystick UP)
                double leftX = -driverController.getLeftX(); // Drive left with negative X (Joystick LEFT)
                double rightX = -driverController.getRightX(); // Drive counterclockwise with negative X (Joystick LEFT)

                // Apply response curves
                leftY = driveCurve.apply(leftY);
                leftX = driveCurve.apply(leftX);
                rightX = rotationCurve.apply(rightX);

                // Scale to m/s and rad/s and apply to the swerve request
                return drive.withVelocityX(leftY * MaxSpeed)
                            .withVelocityY(leftX * MaxSpeed)
                            .withRotationalRate(rightX * MaxAngularRate);
            })
        );
        superstructure.setDefaultCommand(
            superstructure.setPreset(SuperstructurePreset.STOW_UPPER)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // driverController.back().and(driverController.y()).whileTrue(superstructure.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(superstructure.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(superstructure.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(superstructure.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
                        () -> (gunnerController.leftBumper().getAsBoolean() ? -1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? 1 : 0)
                    )
                    // superstructure.setPresetWithAutoCenter(preset)
            ));
        }
        // Alternative receive entry by the driver
        driverController.rightBumper().onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    superstructure.setDefaultCommand(
                        superstructure.setPreset(SuperstructurePreset.STOW_UPPER)
                    );
                }),
                scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations).until(scrubber::safe),
                superstructure.setPresetWithAutoCenter(SuperstructurePreset.RECEIVE)
            )
        );
        gunnerController.b().onTrue(
            new SequentialCommandGroup(
                scrubber.set(() -> Constants.Scrubber.GEAR_RATIO * SuperstructurePreset.STOW_LOWER.pivotRotations).until(scrubber::safe),
                superstructure.setPresetWithBeltOverride(
                    SuperstructurePreset.STOW_LOWER,
                    () -> (gunnerController.leftBumper().getAsBoolean() ? -1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? 1 : 0),
                    () -> (gunnerController.leftBumper().getAsBoolean() ? -1 : 0) + (gunnerController.rightBumper().getAsBoolean() ? 1 : 0)
                )
            )  
        );

        buttonBox.getTrigger(Button.GO_PRESET).onTrue(superstructure.setPreset(SuperstructurePreset.getCorrespondingGoState(superstructure.getState())));

        drivetrain.registerTelemetry(logger::telemeterize);
        driverController.leftBumper().whileTrue(new AutoPlace(drivetrain, superstructure, scrubber,
            new Node(lChooser.getSelected(), getHex(), leftRightChooser.getSelected(),
            scrubChooser.getSelected() ? AutoPlace.getScrubPose(getHex()) : SuperstructurePreset.MANUAL_OVERRIDE
        ), "", false, false, getBackwards()));
        driverController.y().whileTrue(new AutoPickup(drivetrain, superstructure, scrubber, () -> AutoPickup.getCoralSide(drivetrain.getState().Pose)));
        driverController.x().whileTrue(new AlignRequestToF(drivetrain, superstructure.getToFLeft(), superstructure.getToFRight()));
        // Put upward after receive
        new Trigger(() -> superstructure.getState() == SuperstructurePreset.RECEIVE)
            .and(superstructure.getCoralSensorIntake())
            // .and(superstructure.getCoralSensorPlace())
            .and(() -> !DriverStation.isAutonomous())
            .onTrue(new InstantCommand(flipUpTimer::restart));
        new Trigger(() -> superstructure.getState() == SuperstructurePreset.RECEIVE)
            .and(superstructure.getCoralSensorIntake())
            .and(superstructure.getCoralSensorPlace())
            .and(() -> !DriverStation.isAutonomous())
            .and(() -> flipUpTimer.hasElapsed(1))
            .and(driverController.rightBumper().negate())
            .onTrue(
                superstructure.setPresetWithAutoCenter(SuperstructurePreset.STOW_UPPER)
            );

        // Configure cycle timer triggers
        configureCycleTimerTriggers();
    }

    // Efficient median calculation helper
    private void addToMedianCalculation(double value) {
        if (medianMaxHeap.isEmpty() || value <= medianMaxHeap.peek()) {
            medianMaxHeap.add(value);
        } else {
            medianMinHeap.add(value);
        }

        // Rebalance heaps if needed
        if (medianMaxHeap.size() > medianMinHeap.size() + 1) {
            medianMinHeap.add(medianMaxHeap.poll());
        } else if (medianMinHeap.size() > medianMaxHeap.size()) {
            medianMaxHeap.add(medianMinHeap.poll());
        }

        // Calculate median
        if (medianMaxHeap.size() == medianMinHeap.size()) {
            medianCycleTime = (medianMaxHeap.peek() + medianMinHeap.peek()) / 2.0;
        } else {
            medianCycleTime = medianMaxHeap.peek();
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void simulationPeriodic() {
        // Called every robot simulation period
    }

    public void robotPeriodic() {
        buttonBox.sendMessage();

        // Rebinds for reef assist
        if (lastReef != getHex() || lastBackwards != getBackwards()) {
            rebindAutoPlace.run();
            lastReef = getHex();
            lastBackwards = getBackwards();
        }

        // Update Field2d object
        field.setRobotPose(drivetrain.getState().Pose);
        boolean left = leftRightChooser.getSelected() == Side.LEFT;
        Pose2d ghostPose = switch (getHex()) {
            case ONE -> left ? AutoPlace.ONE_LEFT : AutoPlace.ONE_RIGHT;
            case TWO -> left ? AutoPlace.TWO_LEFT : AutoPlace.TWO_RIGHT;
            case THREE -> left ? AutoPlace.THREE_LEFT : AutoPlace.THREE_RIGHT;
            case FOUR -> left ? AutoPlace.FOUR_LEFT : AutoPlace.FOUR_RIGHT;
            case FIVE -> left ? AutoPlace.FIVE_LEFT : AutoPlace.FIVE_RIGHT;
            case SIX -> left ? AutoPlace.SIX_LEFT : AutoPlace.SIX_RIGHT;
        };
        // Flips to the other side of the field if it determines we need it
        ghostPose = Functions.mirrorPoseToRed(ghostPose);
        field.getObject("PathTarget").setPose(ghostPose);

        // Get the active path from the network tables and set it onto the field object
        // Disabled for CPU / network bandwidth reasons
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // NetworkTable table = inst.getTable("/PathPlanner");
        // NetworkTableEntry activePathEntry = table.getEntry("activePath");

        // byte[] activePathBytes = activePathEntry.getRaw(new byte[0]);

        // Deserialize the raw bytes into Pose2d objects
        // List<Pose2d> activePath = new ArrayList<>();
        // ByteBuffer buffer = ByteBuffer.wrap(activePathBytes).order(ByteOrder.LITTLE_ENDIAN);
        // while (buffer.remaining() >= 24) {
            // double x = buffer.getDouble();
            // double y = buffer.getDouble();
            // double theta = buffer.getDouble();
            // activePath.add(new Pose2d(new Translation2d(x, y), new Rotation2d(theta)));
        // }

        // if (!activePath.isEmpty()) {
            // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(activePath, new TrajectoryConfig(2.0, 2.0));
            // field.getObject("PathTrajectory").setTrajectory(trajectory);
            // field.getObject("PathTrajectory").setPoses(activePath);
        // }

        updateDashboard();
        updateDriveSettings();
    }

    /**
     * Initialize SmartDashboard tuning parameters
     */
    private void initializeDashboardTuning() {
        // Driver control tuning - only if enabled
        if (Constants.ENABLE_DRIVER_CONTROLLER_TUNING) {
            // Drive request type chooser
            driveRequestTypeChooser.setDefaultOption("Open Loop Voltage", DriveRequestType.OpenLoopVoltage);
            driveRequestTypeChooser.addOption("Velocity", DriveRequestType.Velocity);
            SmartDashboard.putData(DRIVER_TUNING_BASE_PATH + "Drive Request Type", driveRequestTypeChooser);

            // Drive curve chooser
            for (JoystickResponseCurve.CurveType curveType : JoystickResponseCurve.CurveType.values()) {
                if (curveType == JoystickResponseCurve.CurveType.LINEAR) {
                    driveCurveChooser.setDefaultOption(curveType.name(), curveType);
                } else {
                    driveCurveChooser.addOption(curveType.name(), curveType);
                }
            }
            SmartDashboard.putData(DRIVER_TUNING_BASE_PATH + "Drive Curve Type", driveCurveChooser);

            // Rotation curve chooser
            for (JoystickResponseCurve.CurveType curveType : JoystickResponseCurve.CurveType.values()) {
                if (curveType == JoystickResponseCurve.CurveType.LINEAR) {
                    rotationCurveChooser.setDefaultOption(curveType.name(), curveType);
                } else {
                    rotationCurveChooser.addOption(curveType.name(), curveType);
                }
            }
            SmartDashboard.putData(DRIVER_TUNING_BASE_PATH + "Rotation Curve Type", rotationCurveChooser);

            // Shared parameters for drive and rotation curves
            SmartDashboard.putNumber(DRIVER_TUNING_BASE_PATH + "Curve Scale Factor", curveScale);
            SmartDashboard.putNumber(DRIVER_TUNING_BASE_PATH + "Dual Rate Breakpoint", dualRateBreakpoint);

            // JoystickResponseCurve deadbands
            SmartDashboard.putNumber(DRIVER_TUNING_BASE_PATH + "Joystick Trans Deadband", translationDeadband);
            SmartDashboard.putNumber(DRIVER_TUNING_BASE_PATH + "Joystick Rot Deadband", rotationDeadband);

            // DriveRequest deadbands (physical units)
            SmartDashboard.putNumber(DRIVER_TUNING_BASE_PATH + "Drive Request Trans Deadband", driveRequestTransDeadband);
            SmartDashboard.putNumber(DRIVER_TUNING_BASE_PATH + "Drive Request Rot Deadband", driveRequestRotDeadband);
        }
    }

    /**
     * Update drive settings based on SmartDashboard values
     */
    public void updateDriveSettings() {
        // Skip if tuning is disabled
        if (!Constants.ENABLE_DRIVER_CONTROLLER_TUNING) {
            return;
        }

        // Get values from dashboard
        DriveRequestType requestType = driveRequestTypeChooser.getSelected();
        JoystickResponseCurve.CurveType driveCurveType = driveCurveChooser.getSelected();
        JoystickResponseCurve.CurveType rotationCurveType = rotationCurveChooser.getSelected();
        double newCurveScale = SmartDashboard.getNumber(DRIVER_TUNING_BASE_PATH + "Curve Scale Factor", curveScale);
        double newBreakpoint = SmartDashboard.getNumber(DRIVER_TUNING_BASE_PATH + "Dual Rate Breakpoint", dualRateBreakpoint);

        // Get values from dashboard
        double newJoystickTransDB = SmartDashboard.getNumber(DRIVER_TUNING_BASE_PATH + "Joystick Trans Deadband", translationDeadband);
        double newJoystickRotDB = SmartDashboard.getNumber(DRIVER_TUNING_BASE_PATH + "Joystick Rot Deadband", rotationDeadband);
        double newDriveTransDB = SmartDashboard.getNumber(DRIVER_TUNING_BASE_PATH + "Drive Request Trans Deadband", driveRequestTransDeadband);
        double newDriveRotDB = SmartDashboard.getNumber(DRIVER_TUNING_BASE_PATH + "Drive Request Rot Deadband", driveRequestRotDeadband);

        // Update drive request if type or deadbands changed
        if (requestType != currentDriveRequestType ||
            Math.abs(newDriveTransDB - driveRequestTransDeadband) > 0.001 ||
            Math.abs(newDriveRotDB - driveRequestRotDeadband) > 0.001) {

            drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(requestType)
                .withDeadband(MaxSpeed * newDriveTransDB)
                .withRotationalDeadband(MaxAngularRate * newDriveRotDB);

            currentDriveRequestType = requestType;
            driveRequestTransDeadband = newDriveTransDB;
            driveRequestRotDeadband = newDriveRotDB;
        }

        // Update drive request if type or deadbands changed
        if (requestType != currentDriveRequestType ||
            Math.abs(newDriveTransDB - driveRequestTransDeadband) > 0.001 ||
            Math.abs(newDriveRotDB - driveRequestRotDeadband) > 0.001) {

            drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(requestType)
                .withDeadband(MaxSpeed * newDriveTransDB)
                .withRotationalDeadband(MaxAngularRate * newDriveRotDB);

            currentDriveRequestType = requestType;
            driveRequestTransDeadband = newDriveTransDB;
            driveRequestRotDeadband = newDriveRotDB;
        }

        // Update drive curve if needed
        if (driveCurveType != currentDriveCurveType ||
            translationDeadband != newJoystickTransDB ||
            curveScale != newCurveScale ||
            dualRateBreakpoint != newBreakpoint) {

            driveCurve = new JoystickResponseCurve(
                driveCurveType,
                newJoystickTransDB,
                // NOTE: Following parameters are shared with rotation curve
                newBreakpoint,
                newCurveScale
            );
            currentDriveCurveType = driveCurveType;
            translationDeadband = newJoystickTransDB;
        }

        // Update rotation curve if needed
        if (rotationCurveType != currentRotationCurveType ||
            rotationDeadband != newJoystickRotDB ||
            curveScale != newCurveScale ||
            dualRateBreakpoint != newBreakpoint) {

            rotationCurve = new JoystickResponseCurve(
                rotationCurveType,
                newJoystickRotDB,
                // NOTE: Following parameters are shared with drive curve
                newBreakpoint,
                newCurveScale
            );
            currentRotationCurveType = rotationCurveType;
            rotationDeadband = newJoystickRotDB;
        }
        // Update shared parameters
        curveScale = newCurveScale;
        dualRateBreakpoint = newBreakpoint;
    }

    private HexSide getHex() {
        return reefChoiceAssist.getSelected() ? AutoPlace.chooseReefSide(drivetrain.getState().Pose) : hexSideChooser.getSelected();
    }

    private boolean getBackwards() {
        // return /*backwardsChooser.getSelected() &&*/ Math.abs(drivetrain.getState().Pose.getRotation().minus(getHex().leftPlace.getRotation()).getDegrees()) > 90;
        return false;
    }
}
