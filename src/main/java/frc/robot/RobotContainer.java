// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private double MaxAngularRate = 3.0 * Math.PI; // 1.5 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driver = new CommandXboxController(0); // My joystick for Driver
    private final CommandXboxController operator  = new CommandXboxController(1); // My joystick for Operator
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final Arm arm = new Arm(); // Arm subsystem
    private final Climber climber = new Climber(); // Climber subsystem
    private final Vision vision = new Vision();
    private final OurShuffleboard shuffleboard = new OurShuffleboard(drivetrain, arm, vision);
    private final AddressableLED leds = new AddressableLED(0);
    private final AddressableLEDBuffer ledData = new AddressableLEDBuffer(12);

    private final Trigger armBottomTrigger = new Trigger(() -> arm.getArmBottomLimit());
    private final Trigger wristTopTrigger = new Trigger(() -> arm.getWristTopLimit());
    private final Trigger lineBreakTrigger = new Trigger(() -> arm.getLineBreak());

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // I want field-centric
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop
    //private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        // reset the field-centric heading on left bumper press
        driver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        /*driver.leftBumper().whileTrue(Commands.startEnd(
          () -> {
            MaxSpeed = 2.5;
            MaxAngularRate = 1.0 * Math.PI;
          }, 
          () -> {
            MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
            MaxAngularRate = 3.0 * Math.PI;
          }
        ));*/
        driver.rightBumper().whileTrue(Commands.either(
            Commands.startEnd(
                () -> arm.setIntakePercent(1.0),
                () -> arm.setIntakePercent(0.0))
                .onlyIf(() -> arm.getShooterMotorVelocity() > 15.0),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.6),
                () -> arm.setIntakePercent(0.0))
            /*drivetrain.applyRequest(() -> {
                arm.setIntakePercent(1.0);
                vision.refresh();
                if(vision.hasNote()) {
                    PhotonTrackedTarget bestNote = vision.bestNote();
                    double bestNoteYaw = bestNote.getYaw();
                    return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                                .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                                .withRotationalRate(-Units.degreesToRadians(bestNoteYaw) * MaxAngularRate * 0.50);
                } else {
                    return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                        .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                        .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate);
                }
            })*/.until(() -> arm.getLineBreak()),
            () -> arm.getLineBreak()));
        driver.rightTrigger().onTrue(Commands.sequence(
            arm.spinWheelsCommand(65.0),
            Commands.runOnce(() -> arm.setIntakePercent(1.0)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0);
            })
        ));

        // Outtake
        driver.b().whileTrue(Commands.startEnd(
            () -> arm.setIntakePercent(-0.5), 
            () -> arm.setIntakePercent(0.0))
        );
        driver.y().onTrue(Commands.sequence(
            Commands.runOnce(() -> arm.setIntakePercent(-0.3)),
            Commands.waitSeconds(0.1),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.3), 
                () -> arm.setIntakePercent(0.0))
                .until(() -> arm.getLineBreak())
        ));

        // Tracking shot
        driver.leftTrigger().whileTrue(Commands.repeatingSequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(65.0)),
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(-8.8),
            drivetrain.applyRequest(() -> {
                vision.refresh();
                if (vision.hasAprilTag()) {
                    PhotonTrackedTarget currentTag = vision.getAprilTag(4);
                    if (currentTag == null) {
                        currentTag = vision.getAprilTag(7);
                    }

                    if (currentTag != null) {
                        double distance = vision.getAprilTagDistance(currentTag);
                        if (distance > 3.9) {
                            arm.setShooterMotorVelocity(75.0);
                        } else {
                            arm.setShooterMotorVelocity(65.0);
                        }
                        arm.setWristPosition(vision.getShootingAngle(distance));
                        if (currentTag.getYaw() < 0.5 && currentTag.getYaw() > -0.5) {
                            for (int i = 0; i < ledData.getLength(); i++) {
                                ledData.setRGB(i, 0, 0, 255);
                            }
                            leds.setData(ledData);
                        }
                        return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                            .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                            .withRotationalRate(-Units.degreesToRadians(currentTag.getYaw()) * MaxAngularRate * 1.9);
                    }
                }

                for (int i = 0; i < ledData.getLength(); i++) {
                    ledData.setRGB(i, 0, 255, 0);
                }
                leds.setData(ledData);
                return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                    .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                    .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate);
            }).finallyDo(() -> {
                for (int i = 0; i < ledData.getLength(); i++) {
                    ledData.setRGB(i, 255, 0, 0);
                }
                leds.setData(ledData);
            })
        ));

        // Note tracking
        /*driver.leftBumper().whileTrue(Commands.sequence(Commands.waitSeconds(0.5), drivetrain.applyRequest(() -> {
            vision.refresh();
            if(vision.hasNote()) {
                PhotonTrackedTarget bestNote = vision.bestNote();
                double bestNoteYaw = bestNote.getYaw();
                return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                            .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                            .withRotationalRate(-Units.degreesToRadians(bestNoteYaw) * MaxAngularRate * 0.50);
            } else {
                return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                    .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                    .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate);
            }
        })));*/
        driver.leftBumper().whileTrue(Commands.either(
            Commands.startEnd(
                () -> arm.setIntakePercent(1.0),
                () -> arm.setIntakePercent(0.0))
                .onlyIf(() -> arm.getShooterMotorVelocity() > 15.0),
            drivetrain.applyRequest(() -> {
                if (arm.getLineBreak()) {
                    arm.setIntakePercent(0.0);
                } else {
                    arm.setIntakePercent(0.6);
                }
                vision.refresh();
                if(vision.hasNote()) {
                    PhotonTrackedTarget bestNote = vision.bestNote();
                    double bestNoteYaw = bestNote.getYaw();
                    return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                                .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                                .withRotationalRate(-Units.degreesToRadians(bestNoteYaw) * MaxAngularRate * 0.50);
                } else {
                    return drive.withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed)
                        .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed)
                        .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate);
                }
            }).finallyDo(() -> arm.setIntakePercent(0.0)),
            () -> arm.getLineBreak()));

        operator.leftTrigger().onTrue(Commands.sequence(
          Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
          arm.moveArmPositionCommand(19.0),
          Commands.waitSeconds(0.2).onlyIf(() -> arm.getArmPosition() > 25.0),
          arm.moveWristPositionCommand(-20.5)
        ));
        operator.a().onTrue(Commands.sequence(
          arm.moveArmPositionCommand(19.0),
          arm.moveWristPositionCommand(-13.5),
          Commands.runOnce(() -> arm.setShooterMotorVelocity(60.0))
        ));
        operator.x().onTrue(Commands.sequence(
          Commands.runOnce(() -> arm.setShooterMotorVelocity(60.0)),
          arm.moveArmPositionCommand(19.0),
          arm.moveWristPositionCommand(-10.5)
        ));
        operator.y().onTrue(Commands.sequence(
          arm.moveArmPositionCommand(30.25),
          arm.moveWristPositionCommand(-1.2),
          Commands.runOnce(() -> arm.setShooterMotorVelocity(25.0))
        ));
        operator.rightTrigger().onTrue(Commands.sequence(
          Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
          arm.moveWristPositionCommand(0.5),
          arm.moveArmPositionCommand(0.0)
        ));
        operator.rightBumper().onTrue(Commands.runOnce(() -> climber.climbExtend(), climber));
        operator.leftBumper().onTrue(Commands.runOnce(() -> climber.climbRetract(), climber));

        armBottomTrigger.onTrue(Commands.runOnce(() -> arm.resetArmMotorPosition(0.0))
            .onlyIf(() -> arm.getArmPosition() < 1.0)
            .ignoringDisable(true));
        wristTopTrigger.onTrue(Commands.runOnce(() -> arm.resetWristMotorPosition(arm.getWristAbsPosition()))
            .ignoringDisable(true));
        lineBreakTrigger.onTrue(Commands.runOnce(() -> {
            for (int i = 0; i < ledData.getLength(); i++) {
                ledData.setRGB(i, 0, 255, 0);
            }
            leds.setData(ledData);
        }).ignoringDisable(true));
        lineBreakTrigger.onFalse(Commands.runOnce(() -> {
            for (int i = 0; i < ledData.getLength(); i++) {
                ledData.setRGB(i, 255, 0, 0);
            }
            leds.setData(ledData);
        }).ignoringDisable(true));

        NamedCommands.registerCommand("MoveWristToPickup", arm.moveWristPositionCommand(0.5));

        NamedCommands.registerCommand("Shoot", Commands.sequence(
            arm.spinWheelsCommand(65.0),
            Commands.runOnce(() -> arm.setIntakePercent(0.6), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0); // Kevin action
            }, arm)
        ));

        NamedCommands.registerCommand("DorpWristAndShoot", Commands.sequence(
            arm.moveWristPositionCommand(5.5),
            arm.spinWheelsCommand(65.0)
        ));

        NamedCommands.registerCommand("Intake", Commands.startEnd(
            () -> arm.setIntakePercent(0.6),
            () -> arm.setIntakePercent(0.0),
            arm
        ).until(() -> arm.getLineBreak()));

        NamedCommands.registerCommand("LiftWrist5.5", arm.moveWristPositionCommand(5.5));
        NamedCommands.registerCommand("LiftWrist3.8", arm.moveWristPositionCommand(3.8));
        NamedCommands.registerCommand("LiftWrist4.2", arm.moveWristPositionCommand(4.2));
        NamedCommands.registerCommand("LiftWrist6.7", arm.moveWristPositionCommand(6.7));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        leds.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, 255, 0, 0);
        }
        leds.setData(ledData);
        leds.start();

        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        shuffleboard.addAutoChooser(autoChooser);
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

    public void setDriveDirection() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            Alliance allianceValue = alliance.get();
            if (allianceValue == Alliance.Red) {
                drivetrain.seedFieldRelative(new Pose2d(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(180.0)));
                drivetrain.seedFieldRelative();
            }
        }
    }
}
