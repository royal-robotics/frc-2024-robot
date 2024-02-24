// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;
  AudioConfigs audioConfigs = new AudioConfigs();

  TalonFX m_armFL = new TalonFX(3); // Front Left ID 3
  TalonFX m_armFR = new TalonFX(9); // Front Right ID 9
  TalonFX m_armBL = new TalonFX(4); // Back Left ID 4
  TalonFX m_armBR = new TalonFX(2); // Back Right ID 2
  TalonFX m_shooter = new TalonFX(5); // Shooter motor ID 5
  TalonFX m_intake = new TalonFX(6); // Intake motor ID 6
  TalonFX m_wristBottom = new TalonFX(7); // wrist Bottom ID 7
  TalonFX m_wristTop = new TalonFX(8); // Wrist top ID 8
  Orchestra m_orchestra = new Orchestra("Kevins Great File.chrp");
  
  // audioConfigs.AllowMusicDurDisable = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    OurShuffleboard shuffleboard = new OurShuffleboard(this);
    m_robotContainer.arm.resetWristMotor();

    m_orchestra.addInstrument(m_armFL);
    System.out.println("The arm motor is " + m_armFL);
    // m_orchestra.addInstrument(m_armFR);
    // m_orchestra.addInstrument(m_armBL);
    // m_orchestra.addInstrument(m_armBR);
    // m_orchestra.addInstrument(m_shooter);
    // m_orchestra.addInstrument(m_intake);
    // m_orchestra.addInstrument(m_wristBottom);
    // m_orchestra.addInstrument(m_wristTop);
    audioConfigs.AllowMusicDurDisable = true;
    m_orchestra.play();
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
    // m_orchestra.addInstrument(m_armFL);
    // System.out.println("The arm motor is " + m_armFL);
    // m_orchestra.addInstrument(m_armFR);
    // m_orchestra.addInstrument(m_armBL);
    // m_orchestra.addInstrument(m_armBR);
    // audioConfigs.AllowMusicDurDisable = true;
    m_orchestra.play();
  }

  @Override
  public void teleopPeriodic() {
    m_orchestra.play();
  }

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
