// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.AudioConfigs;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;
  AudioConfigs audioConfigs = new AudioConfigs();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    audioConfigs.AllowMusicDurDisable = true;
    OurShuffleboard shuffleboard = new OurShuffleboard(this);
    CommandSwerveDrivetrain.m_orchestra.play();
    m_robotContainer.arm.resetWristMotor();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    m_robotContainer.arm.setMotorCoast();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.arm.setMotorBrake();
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
    m_robotContainer.arm.setMotorBrake();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Arm.m_orchestra.play();
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
  public void simulationPeriodic() {}
}
