package frc.robot;

import java.util.Map;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class OurShuffleboard {
    public OurShuffleboard(CommandSwerveDrivetrain drivetrain, Arm arm) {
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.addBoolean("Arm Bottom", () -> arm.getArmBottomLimit()).withPosition(0, 0);
        competitionTab.addBoolean("Wrist Top", () -> arm.getWristTopLimit()).withPosition(1, 0);
        competitionTab.addDouble("Odometry X", () -> drivetrain.odometryX()).withPosition(0, 1);
        competitionTab.addDouble("Odometry Y", () -> drivetrain.odometryY()).withPosition(1, 1);
        competitionTab.addCamera("AprilTag", "cam1", "mjpg:http://10.25.22.32:1182/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(2, 0)
            .withSize(4, 4);
        competitionTab.addCamera("Notes", "cam2", "mjpg:http://10.25.22.32:1184/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(6, 0)
            .withSize(3, 3);

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armTab.addDouble("Arm Position", () -> arm.getArmPosition()).withPosition(0, 2);
        armTab.addDouble("Arm Angle", () -> arm.getArmAngle()).withPosition(1, 2);
        armTab.addDouble("Wrist Position", () -> arm.getWristPosition()).withPosition(3, 2);
        armTab.addDouble("Wrist Angle", () -> arm.getWristAngle()).withPosition(4, 2);
        armTab.addDouble("ShooterRPS", () -> arm.getShooterMotorVelocity()).withPosition(6, 2);
        armTab.addDouble("ShooterRPM", () -> arm.getShooterWheelRPM()).withPosition(7, 2);

        ShuffleboardTab aprilTagTab = Shuffleboard.getTab("AprilTag");
        aprilTagTab.addBoolean("Has Targets", () -> arm.hasAprilTag()).withPosition(0, 0);
        aprilTagTab.addDouble("Target Yaw", () -> arm.getTargetYaw()).withPosition(2, 1);
        aprilTagTab.addDouble("Target X", () -> arm.getTargetX()).withPosition(0, 1);
        aprilTagTab.addDouble("Target Y", () -> arm.getTargetY()).withPosition(1, 1);
        aprilTagTab.addDouble("Distance", () -> {
            PhotonTrackedTarget target = arm.getTrackingTarget();
            if (target != null) {
                return PhotonUtils.calculateDistanceToTargetMeters(
                    Units.inchesToMeters(30.0), 
                    Units.degreesToRadians(-35.0), 
                    Units.inchesToMeters(57.375), 
                    Units.degreesToRadians(target.getPitch()));
            } else {
                return 0.0;
            }
        }).withPosition(3, 1);
        aprilTagTab.addDouble("Drivebase Angle Offset", () -> {
            return drivetrain.getAngle() + 10.0;
        }).withPosition(0, 2);
        aprilTagTab.addDouble("M1 Top", () -> {
            double drivebaseAngle = drivetrain.getAngle() + 10.0;
            return (arm.getTargetY() - Units.inchesToMeters(18.0) * Math.sin(Units.degreesToRadians(drivebaseAngle))) - Units.inchesToMeters(7.30);
        }).withPosition(1, 2);
        aprilTagTab.addDouble("M1 Bottom", () -> {
            double drivebaseAngle = drivetrain.getAngle() + 10.0;
            return (arm.getTargetX() - Units.inchesToMeters(18.0) * Math.cos(Units.degreesToRadians(drivebaseAngle))) - Units.inchesToMeters(10.43);
        }).withPosition(2, 2);
        aprilTagTab.addDouble("M1", () -> {
            double drivebaseAngle = drivetrain.getAngle() + 10.0;
            double m1Top = (arm.getTargetY() - Units.inchesToMeters(18.0) * Math.sin(Units.degreesToRadians(drivebaseAngle))) - Units.inchesToMeters(7.30);
            double m1Bottom = (arm.getTargetX() - Units.inchesToMeters(18.0) * Math.cos(Units.degreesToRadians(drivebaseAngle))) - Units.inchesToMeters(10.43);
            return m1Top / m1Bottom;
        }).withPosition(3, 2);
        aprilTagTab.addDouble("M2", () -> {
            return arm.getTargetY() / arm.getTargetX();
        }).withPosition(4, 2);
        aprilTagTab.addDouble("Total Adjustment", () -> {
            double drivebaseAngle = drivetrain.getAngle() + 10.0;
            double m1Top = (arm.getTargetY() - Units.inchesToMeters(18.0) * Math.sin(Units.degreesToRadians(drivebaseAngle))) - Units.inchesToMeters(7.30);
            double m1Bottom = (arm.getTargetX() - Units.inchesToMeters(18.0) * Math.cos(Units.degreesToRadians(drivebaseAngle))) - Units.inchesToMeters(10.43);
            double m1 = m1Top / m1Bottom;
            double m2 = arm.getTargetY() / arm.getTargetX();
            return Math.atan((m1 - m2) / (1.0 + (m1 * m2))) + 10.0;
        }).withPosition(0, 3);
    }

    public void addAutoChooser(SendableChooser<Command> autoChooser){
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add("Autonomous", autoChooser).withPosition(0, 2).withSize(2, 1);
    }
}