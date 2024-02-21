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
        competitionTab.addBoolean("Line Break, Note Loaded", () -> arm.lineBreakSensor.get());
        competitionTab.addBoolean("Arm at Zero", () -> arm.armLimitZero.get());

    }
}