package frc.robot;

import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
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
        aprilTagTab.addDouble("First Target Distance", () -> {
            double distance = 0.0;
            List<PhotonTrackedTarget> targets = arm.getAprilTags();
            for (PhotonTrackedTarget target : targets) {
                int id = target.getFiducialId();
                if (id == 4 || id == 7) {
                    distance = PhotonUtils.calculateDistanceToTargetMeters(
                        Units.inchesToMeters(32.0), 
                        Units.inchesToMeters(57.375), 
                        Units.degreesToRadians(35.0), 
                        Units.degreesToRadians(targets.get(0).getPitch()))
                        * 2 - 1; // Distance off
                }
            }
            return distance;
        }).withPosition(1, 0);
    }

    public void addAutoChooser(SendableChooser<Command> autoChooser){
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add(autoChooser).withPosition(0, 2).withSize(2, 1);
    }
}