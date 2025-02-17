// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // TEMP: Set up LED
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  public Robot() {
    // WPILib logging
    // See https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html
    // DataLogManager.start("/media/sda1/logs");
    DataLogManager.start();
    Epilogue.bind(this);
    m_robotContainer = new RobotContainer();

    // TEMP: Set up LED
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(13);
    m_led.setLength(m_ledBuffer.getLength());
    LEDPattern red = LEDPattern.solid(Color.kGreen);
    red.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    m_robotContainer.simulationPeriodic();
  }
}
