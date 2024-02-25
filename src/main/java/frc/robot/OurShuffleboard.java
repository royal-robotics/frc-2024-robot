package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class OurShuffleboard {
    public OurShuffleboard(CommandSwerveDrivetrain drivetrain, Arm arm) {
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.addBoolean("Line Break", () -> arm.getLineBreak()).withPosition(0, 0);
        competitionTab.addBoolean("Arm Bottom", () -> arm.getArmBottomLimit()).withPosition(1, 0);
        competitionTab.addBoolean("Wrist Top", () -> arm.getWristTopLimit()).withPosition(1, 1);
        competitionTab.addDouble("Arm Position", () -> arm.getArmPosition()).withPosition(2, 0);
        competitionTab.addDouble("Arm Angle", () -> arm.getArmAngle()).withPosition(2, 1);
        competitionTab.addDouble("Wrist Position", () -> arm.getWristPosition()).withPosition(3, 0);
        competitionTab.addDouble("Wrist Angle", () -> arm.getWristAngle()).withPosition(3, 1);
        competitionTab.addDouble("Wrist Abs Pos", () -> arm.getWristAbsPosition()).withPosition(4, 0);
        competitionTab.addDouble("Wrist Abs Angle", () -> arm.getWristAbsAngle()).withPosition(4, 1);
        competitionTab.addDouble("Shoot Motor RPM", () -> arm.getShooterMotorVelocity()).withPosition(5, 0);
        competitionTab.addDouble("Shoot Wheel RPM", () -> arm.getShooterWheelRPM()).withPosition(5, 1);
        competitionTab.addDouble("Odometry X", () -> drivetrain.odometryX()).withPosition(3, 2);
        competitionTab.addDouble("Odometry Y", () -> drivetrain.odometryY()).withPosition(4, 2);

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armTab.addDouble("Arm Position", () -> arm.getArmPosition()).withPosition(0, 2);
        armTab.addDouble("Arm Angle", () -> arm.getArmAngle()).withPosition(1, 2);
        armTab.addDouble("Wrist Position", () -> arm.getWristPosition()).withPosition(3, 2);
        armTab.addDouble("Wrist Angle", () -> arm.getWristAngle()).withPosition(4, 2);
        armTab.addDouble("ShooterRPS", () -> arm.getShooterMotorVelocity()).withPosition(6, 2);
        armTab.addDouble("ShooterRPM", () -> arm.getShooterWheelRPM()).withPosition(7, 2);
    }

    public void addAutoChooser(SendableChooser<Command> autoChooser){
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add(autoChooser).withPosition(0, 2).withSize(2, 1);
    }
}