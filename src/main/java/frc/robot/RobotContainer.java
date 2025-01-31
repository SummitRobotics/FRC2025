// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoPlace.HexSide;
import frc.robot.commands.AutoPlace.Node;
import frc.robot.commands.AutoPlace.Side;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.ButtonBox;
import frc.robot.oi.ButtonBox.Button;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Scrubber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructurePreset;
import frc.robot.utilities.lists.Constants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.825).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController driverXBox = new CommandXboxController(Constants.OI.DRIVER_XBOX);
    private final CommandXboxController gunnerXBox = new CommandXboxController(Constants.OI.GUNNER_XBOX);
    private final ButtonBox buttonBox = new ButtonBox(Constants.OI.BUTTON_BOX);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Superstructure superstructure = new Superstructure();
    public final Scrubber scrubber = new Scrubber();
    
    // Path follower
    private final SendableChooser<Command> autoChooser;
    // private final SendableChooser<SuperstructurePreset> lChooser;
    // private final SendableChooser<HexSide> hexSideChooser;
    // private final SendableChooser<Side> leftRightChooser;

    // Node chooser
    private Side leftRight = Side.LEFT;
    private SuperstructurePreset l = SuperstructurePreset.L1;
    private HexSide hexSide = HexSide.ONE;

    private Command generateSelectCommand(HexSide hexSide, Side leftRight) {
        return new InstantCommand(() -> {
            this.hexSide = hexSide;
            this.leftRight = leftRight;
            driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(l, hexSide, leftRight)));
        });
    }

    private final Command generateSelectCommand(SuperstructurePreset l) {
        return new InstantCommand(() -> {
            this.l = l;
            driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(l, hexSide, leftRight)));
        });

    }

    private final Command
        selectOneLeft = generateSelectCommand(HexSide.ONE, Side.LEFT),
        selectOneRight = generateSelectCommand(HexSide.ONE, Side.RIGHT),
        selectTwoLeft = generateSelectCommand(HexSide.TWO, Side.LEFT),
        selectTwoRight = generateSelectCommand(HexSide.TWO, Side.RIGHT),
        selectThreeLeft = generateSelectCommand(HexSide.THREE, Side.LEFT),
        selectThreeRight = generateSelectCommand(HexSide.THREE, Side.RIGHT),
        selectFourLeft = generateSelectCommand(HexSide.FOUR, Side.LEFT),
        selectFourRight = generateSelectCommand(HexSide.FOUR, Side.RIGHT),
        selectFiveLeft = generateSelectCommand(HexSide.FIVE, Side.LEFT),
        selectFiveRight = generateSelectCommand(HexSide.FIVE, Side.RIGHT),
        selectSixLeft = generateSelectCommand(HexSide.SIX, Side.LEFT),
        selectSixRight = generateSelectCommand(HexSide.SIX, Side.RIGHT),
        selectL1 = generateSelectCommand(SuperstructurePreset.L1),
        selectL2 = generateSelectCommand(SuperstructurePreset.L2),
        selectL3 = generateSelectCommand(SuperstructurePreset.L3),
        selectL4 = generateSelectCommand(SuperstructurePreset.L4);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // lChooser = new SendableChooser<SuperstructurePreset>();
        // hexSideChooser = new SendableChooser<HexSide>();
        // leftRightChooser = new SendableChooser<Side>();
        // lChooser.setDefaultOption("L1", SuperstructurePreset.L1);
        // lChooser.addOption("L2", SuperstructurePreset.L2);
        // lChooser.addOption("L3", SuperstructurePreset.L3);
        // lChooser.addOption("L4", SuperstructurePreset.L4);
        // hexSideChooser.setDefaultOption("1", AutoPlace.HexSide.ONE);
        // hexSideChooser.addOption("2", AutoPlace.HexSide.TWO);
        // hexSideChooser.addOption("3", AutoPlace.HexSide.THREE);
        // hexSideChooser.addOption("4", AutoPlace.HexSide.FOUR);
        // hexSideChooser.addOption("5", AutoPlace.HexSide.FIVE);
        // hexSideChooser.addOption("6", AutoPlace.HexSide.SIX);
        // leftRightChooser.setDefaultOption("Left", AutoPlace.Side.LEFT);
        // leftRightChooser.addOption("Right", AutoPlace.Side.RIGHT);
        // These weren't changing the bound command properly before this got added, so it seems like the rebinds are necessary.
        // lChooser.onChange((SuperstructurePreset l) -> {
            // driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(l, hexSideChooser.getSelected(), leftRightChooser.getSelected())));
        // });
        // hexSideChooser.onChange((HexSide hexSide) -> {
            // driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(lChooser.getSelected(), hexSide, leftRightChooser.getSelected())));
        // });
        // leftRightChooser.onChange((leftRight) -> {
            // driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(lChooser.getSelected(), hexSideChooser.getSelected(), leftRight)));
        // });
        // SmartDashboard.putData("L Chooser", lChooser);
        // SmartDashboard.putData("Hex Side Chooser", hexSideChooser);
        // SmartDashboard.putData("Left-Right Chooser", leftRightChooser);
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("One Left", selectOneLeft);
        SmartDashboard.putData("One Right", selectOneRight);
        SmartDashboard.putData("Two Left", selectTwoLeft);
        SmartDashboard.putData("Two Right", selectTwoRight);
        SmartDashboard.putData("Three Left", selectThreeLeft);
        SmartDashboard.putData("Three Right", selectThreeRight);
        SmartDashboard.putData("Four Left", selectFourLeft);
        SmartDashboard.putData("Four Right", selectFourRight);
        SmartDashboard.putData("Five Left", selectFiveLeft);
        SmartDashboard.putData("Five Right", selectFiveRight);
        SmartDashboard.putData("Six Left", selectSixLeft);
        SmartDashboard.putData("Six Right", selectSixRight);
        SmartDashboard.putData("L1", selectL1);
        SmartDashboard.putData("L2", selectL2);
        SmartDashboard.putData("L3", selectL3);
        SmartDashboard.putData("L4", selectL4);
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
                drive.withVelocityX(-driverXBox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXBox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverXBox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverXBox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXBox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverXBox.getLeftY(), -driverXBox.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverXBox.back().and(driverXBox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverXBox.back().and(driverXBox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverXBox.start().and(driverXBox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverXBox.start().and(driverXBox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverXBox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Superstructure MOs
        buttonBox.getTrigger(Button.MO_PRESET).whileTrue(superstructure.setManual(
            () -> gunnerXBox.getLeftTriggerAxis() * Constants.Elevator.MAX_ROTATIONS,
            () -> Constants.Manipulator.MIN_RADIANS + gunnerXBox.getRightTriggerAxis() * (Constants.Manipulator.MAX_RADIANS - Constants.Manipulator.MIN_RADIANS),
            () -> (gunnerXBox.povUp().getAsBoolean() ? 1 : 0) + (gunnerXBox.povDown().getAsBoolean() ? -1 : 0),
            () -> (gunnerXBox.povLeft().getAsBoolean() ? 1 : 0) + (gunnerXBox.povRight().getAsBoolean() ? -1 : 0)
        ));
        // Scrubber MOs
        buttonBox.getTrigger(Button.MO_PRESET).whileTrue(scrubber.set(() -> gunnerXBox.x().getAsBoolean() ? Constants.Scrubber.MAX_ROTATIONS : 0));

        drivetrain.registerTelemetry(logger::telemeterize);
        // driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, new Node(lChooser.getSelected(), hexSideChooser.getSelected(), leftRightChooser.getSelected())));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void robotPeriodic() {
        buttonBox.sendMessage();
    }
}
