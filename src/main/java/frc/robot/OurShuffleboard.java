package frc.robot;

import java.util.Map;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class OurShuffleboard {
    public OurShuffleboard(CommandSwerveDrivetrain drivetrain, Arm arm, Vision vision) {
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.addBoolean("Arm Bottom", () -> arm.getArmBottomLimit()).withPosition(0, 0);
        competitionTab.addBoolean("Wrist Top", () -> arm.getWristTopLimit()).withPosition(1, 0);
        competitionTab.addCamera("AprilTag", "cam1", "mjpg:http://10.25.22.32:1184/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(2, 0)
            .withSize(4, 4);
        competitionTab.addCamera("Notes", "cam2", "mjpg:http://10.25.22.32:1182/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(6, 0)
            .withSize(3, 3);

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armTab.addDouble("Arm Position", () -> arm.getArmPosition()).withPosition(0, 2);
        armTab.addDouble("Arm Angle", () -> arm.getArmAngle()).withPosition(1, 2);
        armTab.addDouble("Wrist Position", () -> arm.getWristPosition()).withPosition(3, 2);
        armTab.addDouble("Wrist Angle", () -> arm.getWristAngle()).withPosition(4, 2);
        armTab.addDouble("Wrist Abs Position", () -> arm.getWristAbsPosition()).withPosition(3, 3);
        armTab.addDouble("Wrist Abs Angle", () -> arm.getWristAbsAngle()).withPosition(4, 3);
        armTab.addDouble("ShooterRPS", () -> arm.getShooterMotorVelocity()).withPosition(6, 2);
        armTab.addDouble("ShooterRPM", () -> arm.getShooterWheelRPM()).withPosition(7, 2);

        ShuffleboardTab aprilTagTab = Shuffleboard.getTab("AprilTag");
        aprilTagTab.addBoolean("Has Tag", () -> vision.hasAprilTag()).withPosition(0, 0);
        aprilTagTab.addDouble("Tag Distance", () -> {
            PhotonTrackedTarget aprilTag = vision.getAprilTag(7);
            if (aprilTag != null) {
                return vision.getAprilTagDistance(aprilTag);
            } else {
                aprilTag = vision.getAprilTag(4);
                if (aprilTag != null) {
                    return vision.getAprilTagDistance(aprilTag);
                }
            }

            return 0.0;
        });
    }

    public void addAutoChooser(SendableChooser<Command> autoChooser){
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.add("Autonomous", autoChooser).withPosition(0, 2).withSize(2, 1);
    }
}