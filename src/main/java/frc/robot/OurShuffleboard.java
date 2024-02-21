package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import frc.robot.autonomous.AutoModeSelector;
// import frc.robot.sensors.Limelight;
import frc.robot.subsystems.*;

public class OurShuffleboard {
    public OurShuffleboard(Robot robot) {
        Arm arm = robot.m_robotContainer.arm;
        ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
        competitionTab.addDouble("Line Break, Note Loaded", () -> arm.lineBreakSensor.getVoltage()).withPosition(0, 0);
        competitionTab.addBoolean("Arm at Zero", () -> arm.armLimitZero.get()).withPosition(1, 0);
        competitionTab.addDouble("Arm position", () -> arm.armPosition.getValue()).withPosition(2, 0);;
        competitionTab.addDouble("wrist position", () -> arm.wristPosition.getValue()).withPosition(3, 0);;
    }
}