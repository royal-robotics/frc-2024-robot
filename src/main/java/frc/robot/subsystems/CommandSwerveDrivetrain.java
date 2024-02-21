package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    AudioConfigs audioConfigs = new AudioConfigs();

    Orchestra m_orchestra = new Orchestra("Kevins Great File.chrp");

    SwerveRequest.ApplyChassisSpeeds autoRequestChasisSpeed = new SwerveRequest.ApplyChassisSpeeds();

    // public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
    //     super(driveTrainConstants, OdometryUpdateFrequency, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // Add a single device to the orchestra
        m_orchestra.addInstrument(this.Modules[0].getDriveMotor());
        System.out.println(this.Modules[0].getDriveMotor());
        m_orchestra.addInstrument(this.Modules[0].getSteerMotor());
        m_orchestra.addInstrument(this.Modules[1].getDriveMotor());
        m_orchestra.addInstrument(this.Modules[1].getSteerMotor());
        m_orchestra.addInstrument(this.Modules[2].getDriveMotor());
        m_orchestra.addInstrument(this.Modules[2].getSteerMotor());
        m_orchestra.addInstrument(this.Modules[3].getDriveMotor());
        m_orchestra.addInstrument(this.Modules[3].getSteerMotor());

        // Attempt to load the chrp
        // StatusCode status = m_orchestra.loadMusic("Kevins Great File.chrp"); // Moved to Object Declaration

        // if (!status.isOK()) {
        // // log error
        //    System.out.println("error in swerve!");
        // }

        audioConfigs.AllowMusicDurDisable = false;

        m_orchestra.play();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            () -> this.m_odometry.getEstimatedPosition(), // Robot pose supplier
            this::resetPose,
            () -> this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::autoRequestChasisSpeed,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public void resetPose(Pose2d pose) {
        this.m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose); // Method to reset odometry (will be called if your auto has a starting pose)
    }

    public void autoRequestChasisSpeed(ChassisSpeeds reqAutoChassisSpeeds) {
        this.setControl(autoRequestChasisSpeed.withSpeeds(reqAutoChassisSpeeds)); // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
     // () -> this.applyRequest(() -> autoRequestChasisSpeed), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
