package frc.robot;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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
        aprilTagTab.addInteger("April Tag ID", () -> arm.aprilTagId()).withPosition(1, 0);
        aprilTagTab.addDouble("Angle", () -> arm.aprilTagAngle()).withPosition(2, 0);
        aprilTagTab.addDouble("Transform X", () -> arm.aprilTagTransformX()).withPosition(0, 1);
        aprilTagTab.addDouble("Transform Y", () -> arm.aprilTagTransformY()).withPosition(1, 1);
        aprilTagTab.addDouble("Transform Z", () -> arm.aprilTagTransformZ()).withPosition(2, 1);
        aprilTagTab.addDouble("April Tag Distance", () -> arm.aprilTagDistance()).withPosition(0, 2);
    }

    public void addAutoChooser(SendableChooser<Command> autoChooser){
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add(autoChooser).withPosition(0, 2).withSize(2, 1);
    }
}