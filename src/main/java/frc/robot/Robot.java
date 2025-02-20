// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
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
  private final AddressableLED m_frontLed;
  private final AddressableLEDBuffer m_frontLedBuffer;
  private final AddressableLED m_leftLed;
  private final AddressableLEDBuffer m_leftLedBuffer;
  private final AddressableLED m_rightLed;
  private final AddressableLEDBuffer m_rightLedBuffer;
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);
  private final LEDPattern m_scrollPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kGreen);
  LEDPattern absolutePattern = m_scrollPattern.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(22), kLedSpacing);
  
  public Robot() {
    // WPILib logging
    // See https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html
    // DataLogManager.start("/media/sda1/logs");
    DataLogManager.start();
    Epilogue.bind(this);
    m_robotContainer = new RobotContainer();

    // TEMP: Set up LED
    m_frontLed = new AddressableLED(0);
    m_frontLedBuffer = new AddressableLEDBuffer(13);
    m_frontLed.setLength(m_frontLedBuffer.getLength());
    LEDPattern red = LEDPattern.solid(Color.kGreen);
    red.applyTo(m_frontLedBuffer);
    m_frontLed.setData(m_frontLedBuffer);
    m_frontLed.start();
    
    m_leftLed = new AddressableLED(1);
    m_leftLedBuffer = new AddressableLEDBuffer(27);
    m_leftLed.setLength(m_leftLedBuffer.getLength());
    m_leftLed.setData(m_leftLedBuffer);
    m_leftLed.start();

    m_rightLed = new AddressableLED(2);
    m_rightLedBuffer = new AddressableLEDBuffer(27);
    m_rightLed.setLength(m_rightLedBuffer.getLength());
    m_rightLed.setData(m_rightLedBuffer);
    m_rightLed.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.robotPeriodic();
    absolutePattern.applyTo(m_leftLedBuffer);
    absolutePattern.applyTo(m_rightLedBuffer);
    m_leftLed.setData(m_leftLedBuffer);
    m_rightLed.setData(m_rightLedBuffer);


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
