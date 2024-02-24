// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 3 * Math.PI; // 1.5 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(0); // My joystick for Driver
    private final CommandXboxController operator  = new CommandXboxController(1); // My joystick for Operator
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final Arm arm = new Arm(); // Arm subsystem
    //private final Climber climber = new Climber(); // Climber subsystem
    private final OurShuffleboard shuffleboard = new OurShuffleboard(arm);

    private final Trigger armBottomTrigger = new Trigger(() -> arm.getArmBottomLimit());
    private final Trigger wristTopTrigger = new Trigger(() -> arm.getWristTopLimit());

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // I want field-centric
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        driver.a().onTrue(Commands.runOnce(() -> arm.setArmPosition(0.0), arm));
        driver.b().onTrue(Commands.runOnce(() -> arm.setWristPosition(0.0), arm));
        driver.y().whileTrue(Commands.startEnd(
            () -> {
                arm.setIntakePercent(0.5);
                arm.setShooterMotorVelocity(240.0);
            },
            () -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0);
            }, arm)
            .until(() -> arm.getLineBreak()));
        driver.x().whileTrue(Commands.startEnd(
            () -> {
                arm.setIntakePercent(0.5);
                arm.setShooterMotorVelocity(240.0);
            },
            () -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0);
            }, arm));

        //operator.a().whileTrue(Commands.runOnce(() -> climber.climbExtend(), climber));
        //operator.b().whileTrue(Commands.runOnce(() -> climber.climbRetract(), climber));

        operator.leftBumper().onTrue(Commands.runOnce(() -> arm.playMusic(), arm));
        operator.rightBumper().onTrue(Commands.runOnce(() -> arm.stopMusic(), arm));

        armBottomTrigger.onTrue(Commands.runOnce(() -> arm.resetArmMotorPosition(0.0), arm));
        wristTopTrigger.onTrue(Commands.runOnce(() -> arm.resetWristMotorPosition(arm.getWristAbsPosition()), arm));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setArmCoast() {
        arm.setMotorCoast();
    }

    public void setArmBrake() {
        arm.setMotorBrake();
    }
}
