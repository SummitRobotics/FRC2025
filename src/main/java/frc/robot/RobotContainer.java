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

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Node> nodeChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        nodeChooser = new SendableChooser<Node>();
        nodeChooser.addOption("3L", new Node(SuperstructurePreset.L4, HexSide.THREE_BLUE, Side.LEFT));
        nodeChooser.addOption("1R", new Node(SuperstructurePreset.L4, HexSide.ONE_BLUE, Side.RIGHT));
        nodeChooser.setDefaultOption("5L", new Node(SuperstructurePreset.L4, HexSide.FIVE_BLUE, Side.LEFT));
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Node chooser", nodeChooser);
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
        buttonBox.getTrigger(Button.MO_PRESET).whileTrue(scrubber.setManual(() -> gunnerXBox.x().getAsBoolean() ? Constants.Scrubber.MAX_ROTATIONS : 0));

        drivetrain.registerTelemetry(logger::telemeterize);
        driverXBox.x().whileTrue(new AutoPlace(drivetrain, superstructure, nodeChooser.getSelected()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void robotPeriodic() {
        buttonBox.sendMessage();
    }
}
