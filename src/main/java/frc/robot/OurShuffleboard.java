package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Map;

public class OurShuffleboard {
    public OurShuffleboard(CommandSwerveDrivetrain drivetrain, Arm arm, Vision vision) {
        // Competition tab
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        // Arm bottom limit switch light
        competitionTab.addBoolean("Arm Bottom", () -> arm.getArmBottomLimit())
            .withPosition(0, 0);
        // Wrist top limit switch light
        competitionTab.addBoolean("Wrist Top", () -> arm.getWristTopLimit())
            .withPosition(1, 0);
        // AprilTag camera
        competitionTab.addCamera("AprilTag", "cam1", "mjpg:http://10.25.22.11:1184/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(2, 0)
            .withSize(4, 4);
        // Note camera
        competitionTab.addCamera("Notes", "cam2", "mjpg:http://10.25.22.11:1182/?action=stream")
            .withProperties(Map.of("showControls", false))
            .withPosition(6, 0)
            .withSize(3, 3);

        // Arm tab
        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        // Arm position (rotations)
        armTab.addDouble("Arm Position", () -> arm.getArmPosition())
            .withPosition(0, 2);
        // Arm angle (degrees)
        armTab.addDouble("Arm Angle", () -> arm.getArmAngle())
            .withPosition(1, 2);
        // Wrist position (rotations)
        armTab.addDouble("Wrist Position", () -> arm.getWristPosition())
            .withPosition(3, 2);
        // Wrist angle (degrees)
        armTab.addDouble("Wrist Angle", () -> arm.getWristAngle())
            .withPosition(4, 2);
        // Wrist absolute position (rotations)
        armTab.addDouble("Wrist Abs Position", () -> arm.getWristAbsPosition())
            .withPosition(3, 3);
        // Wrist absolute angle (degrees)
        armTab.addDouble("Wrist Abs Angle", () -> arm.getWristAbsAngle())
            .withPosition(4, 3);
        // Shooter motor speed (rotations per second)
        armTab.addDouble("Shooter motor RPS", () -> arm.getShooterMotorVelocity())
            .withPosition(6, 2);
        // Shooter wheel speed (rotations per minute)
        armTab.addDouble("Shooter wheel RPM", () -> arm.getShooterWheelRPM())
            .withPosition(7, 2);
    }

    public void addAutoChooser(SendableChooser<Command> autoChooser) {
        // Competition tab
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        // Autonomous chooser
        competitionTab.add("Autonomous", autoChooser)
            .withPosition(0, 2)
            .withSize(2, 1);
    }
}