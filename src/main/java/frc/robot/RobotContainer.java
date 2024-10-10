// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 5.0 * Math.PI; // 1.5 rotations per second max angular velocity

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
        .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 2.5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop
    //private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private final GenericEntry demoMode;

    private final double passingWheelSpeed = 47.5; // RPS of shooter wheels for passing shots 

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double TopSpin = MaxAngularRate;
                double TopSpeed = MaxSpeed;
                if(demoMode.getBoolean(false)){
                    TopSpeed = MaxSpeed/4;
                    TopSpin = MaxAngularRate/4;
                }
                return drive
                .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * TopSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * TopSpeed) // Drive left with negative X (left)
                .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 3), -driver.getRightX()) * TopSpin); // Drive counterclockwise with negative X (left)
            }
        ));


        // Driver Nickey Controls
        // Driver Controls
        // Left Trigger -- Tracking Shot
        // Right Trigger -- Speaker Shot
        // Left Bumper -- Note Tracking
        // Right Bumper -- Intake
        // Back -- Reset Gyro
        // b -- Outtake
        // y -- Nickey's favorite out-in button

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

        // Intake command
        driver.rightBumper().whileTrue(Commands.either(
            Commands.startEnd(
                () -> arm.setIntakePercent(1.0),
                () -> arm.setIntakePercent(0.0))
                .onlyIf(() -> arm.getShooterMotorVelocity() > 15.0),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.5),
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

        // Speaker Shot
        driver.rightTrigger().onTrue(Commands.sequence(
            // Start intake and shooter motors simultaneously
            arm.spinShooterMotorCommand(75.0),
            Commands.runOnce(() -> arm.setIntakePercent(1.0)),
            // Wait half a second to end intake
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0);
            })
        ));
        
        //Stop Shooter
        driver.povUp().onTrue(arm.runOnce(() ->arm.setShooterMotorVelocity(0.0)));

        // Outtake
        
        /*driver.b().whileTrue(Commands.startEnd(
            () -> arm.setIntakePercent(-0.5), 
            () -> arm.setIntakePercent(0.0))
        );*/
    
        driver.b().onTrue(Commands.either(
            Commands.sequence(
            Commands.either(
                Commands.runOnce(() -> arm.setShooterMotorVelocity(25.0)),
                Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
                () -> arm.getLineBreak()),
            arm.moveArmPositionCommand(19.0),
            Commands.waitSeconds(0.2).onlyIf(() -> arm.getArmPosition() > 25.0),
            arm.moveWristPositionCommand(-20.5)
        ),
            Commands.sequence(
                Commands.runOnce(() -> arm.setIntakePercent(-0.5)),
                Commands.waitSeconds(1),
                Commands.runOnce(() -> arm.setIntakePercent(0.0))
        ),
        () -> demoMode.getBoolean(false)
        ));
    
        // Nickey's Favorite Button
        /*driver.y().onTrue(Commands.sequence(
            Commands.runOnce(() -> arm.setIntakePercent(-0.3)),
            Commands.waitSeconds(0.1),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.3), 
                () -> arm.setIntakePercent(0.0))
                .until(() -> arm.getLineBreak())
        ));*/

        driver.y().onTrue(Commands.either(
            Commands.sequence(
            arm.moveArmPositionCommand(30.25),
            arm.moveWristPositionCommand(-1.2),
            Commands.runOnce(() -> arm.setShooterMotorVelocity(20.0))
        ),
            Commands.sequence(
            Commands.runOnce(() -> arm.setIntakePercent(-0.3)),
            Commands.waitSeconds(0.1),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.3), 
                () -> arm.setIntakePercent(0.0))
                .until(() -> arm.getLineBreak())
        ),
        () -> demoMode.getBoolean(false)
        ));

        // Tracking shot
        driver.leftTrigger().whileTrue(Commands.repeatingSequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(85.0)),
            // Clamshot
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(-8.7),
            drivetrain.applyRequest(() -> {
                vision.refresh();
                if (vision.hasAprilTag()) {
                    // Find AprilTag ID 4 first
                    PhotonTrackedTarget currentTag = vision.getAprilTag(4);
                    double targetYaw = -3.0;

                    // Now find AprilTag ID 7
                    if (currentTag == null) {
                        currentTag = vision.getAprilTag(7);
                        targetYaw = -3.0;
                    }

                    // if AprilTag found, run tracking code
                    if (currentTag != null) {
                        double distance = vision.getAprilTagDistance(currentTag);
                        if (distance > 3.9) {
                            // Faster shooter motor further away
                            arm.setShooterMotorVelocity(75.0);
                        } else {
                            arm.setShooterMotorVelocity(65.0);
                        }
                        arm.setWristPosition(vision.getShootingAngle(distance));
                        
                        // Set LEDs to blue when AprilTag is being tracked
                        if (currentTag.getYaw() < targetYaw + 0.75 && currentTag.getYaw() > targetYaw - 0.75) {
                            for (int i = 0; i < ledData.getLength(); i++) {
                                ledData.setRGB(i, 0, 0, 255);
                            }
                            leds.setData(ledData);
                        }
                            double TopSpeed = MaxSpeed;
                            double TopSpin = MaxAngularRate;
                        if(demoMode.getBoolean(false)){
                         TopSpeed = MaxSpeed;
                         TopSpin = MaxAngularRate/2;
                        }
                        return drive
                            .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * TopSpeed)
                            .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * TopSpeed)
                            .withRotationalRate(-Units.degreesToRadians(currentTag.getYaw() - targetYaw) * TopSpin * 1.9 * (3.0 / 5.0));
                    }
                }

                for (int i = 0; i < ledData.getLength(); i++) {
                    ledData.setRGB(i, 0, 255, 0);
                }
                leds.setData(ledData);
                double TopSpeed = MaxSpeed;
                double TopSpin = MaxAngularRate;
                if(demoMode.getBoolean(false)){
                    TopSpeed = MaxSpeed/3.5;
                    TopSpin = MaxAngularRate/2;
                }
                return drive
                    .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * TopSpeed)
                    .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * TopSpeed)
                    .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 3), -driver.getRightX()) * TopSpin);
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

        // Note Tracking
        driver.leftBumper().whileTrue(Commands.either(
            Commands.startEnd(
                () -> arm.setIntakePercent(1.0),
                () -> arm.setIntakePercent(0.0))
                .onlyIf(() -> arm.getShooterMotorVelocity() > 15.0),
            drivetrain.applyRequest(() -> {
                if (arm.getLineBreak()) {
                    arm.setIntakePercent(0.0);
                } else {
                    arm.setIntakePercent(0.5);
                }
                vision.refresh();
                if(vision.hasNote()) {
                    if (arm.getLineBreak()) {
                        for (int i = 0; i < ledData.getLength(); i++) {
                            ledData.setRGB(i, 0, 255, 0);
                        }
                        leds.setData(ledData);
                    } else {
                        for (int i = 0; i < ledData.getLength(); i++) {
                            ledData.setRGB(i, 255, 55, 0);
                        }
                        leds.setData(ledData);
                    }

                    PhotonTrackedTarget bestNote = vision.bestNote();
                    double bestNoteYaw = bestNote.getYaw();
                     double TopSpeed = MaxSpeed;
                     double TopSpin = MaxAngularRate;
                    if(demoMode.getBoolean(false)){
                    TopSpeed = MaxSpeed/4;
                    TopSpin = MaxAngularRate/2;
                    }
                    return drive
                        .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * TopSpeed)
                        .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * TopSpeed)
                        .withRotationalRate(-Units.degreesToRadians(bestNoteYaw) * TopSpin * 0.50 * (3.0 / 5.0));
                } else {
                    if (arm.getLineBreak()) {
                        for (int i = 0; i < ledData.getLength(); i++) {
                            ledData.setRGB(i, 0, 255, 0);
                        }
                        leds.setData(ledData);
                    } else {
                        for (int i = 0; i < ledData.getLength(); i++) {
                            ledData.setRGB(i, 255, 0, 0);
                        }
                        leds.setData(ledData);
                    }
                     double TopSpeed = MaxSpeed;
                     double TopSpin = MaxAngularRate;
                    if(demoMode.getBoolean(false)){
                    TopSpeed = MaxSpeed/4;
                    TopSpin = MaxAngularRate/2;
                    }
                    return drive
                        .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * TopSpeed)
                        .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * TopSpeed)
                        .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 3), -driver.getRightX()) * TopSpin);
                }
            }).finallyDo(() -> {
                arm.setIntakePercent(0.0);
                if (arm.getLineBreak()) {
                    for (int i = 0; i < ledData.getLength(); i++) {
                        ledData.setRGB(i, 0, 255, 0);
                    }
                    leds.setData(ledData);
                    //spin up after intake
                    arm.setShooterMotorVelocity(40.0);
                } else {
                    for (int i = 0; i < ledData.getLength(); i++) {
                        ledData.setRGB(i, 255, 0, 0);
                    }
                    leds.setData(ledData);
                }
            }),
            () -> arm.getLineBreak()));

        // driver.rightStick().onTrue(Commands.startEnd(() -> MaxAngularRate = 5.0 * Math.PI, () -> MaxAngularRate = 3.0 * Math.PI));
// 
        // Operator Controls
        // Left Trigger -- Clamshell
        // Right Trigger -- Stop shooter motors, reset to ground
        // Left Bumper -- Retract Climber
        // Right Bumper -- Extend Climber
        // a -- Clamshot
        // b -- Source Intake 
        // x -- Passing shot
        // y -- Amp button

        // Clamshell
        operator.leftTrigger().onTrue(Commands.sequence(
            Commands.either(
                Commands.runOnce(() -> arm.setShooterMotorVelocity(30.0)),
                Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
                () -> arm.getLineBreak()),
            arm.moveArmPositionCommand(19.0),
            Commands.waitSeconds(0.2).onlyIf(() -> arm.getArmPosition() > 25.0),
            arm.moveWristPositionCommand(-20.5)
        ));

        //Demo Clamshell
        /*if(demoMode.getBoolean(false)){
            driver.b().onTrue(Commands.sequence(
            Commands.either(
                Commands.runOnce(() -> arm.setShooterMotorVelocity(25.0)),
                Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
                () -> arm.getLineBreak()),
            arm.moveArmPositionCommand(19.0),
            Commands.waitSeconds(0.2).onlyIf(() -> arm.getArmPosition() > 25.0),
            arm.moveWristPositionCommand(-20.5)
        ));
        }*/
    

        // Clamshot
        operator.a().onTrue(Commands.sequence(
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(-13.5),
            Commands.runOnce(() -> arm.setShooterMotorVelocity(55.0))
        ));

        // Source intake
        // operator.b().onTrue(Commands.sequence(
        //     arm.moveArmPositionCommand(19.0),
        //     arm.moveWristPositionCommand(4.5),
        //     Commands.startEnd(() -> arm.setIntakePercent(0.5), () -> arm.setIntakePercent(0.0)).until(() -> arm.getLineBreak())
        // ));

        // Passing Overstage
        operator.b().onTrue(Commands.sequence(
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(-13.5),
            Commands.runOnce(() -> arm.setShooterMotorVelocity(passingWheelSpeed))
        ));

        // Passing Floor
        operator.x().onTrue(Commands.sequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(85.0)),
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(0.0)
        ));

        // Passing Overstage

        operator.povUp().onTrue(Commands.sequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(80.0)),
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(2.2)
        ));
        
        //Demo Pass
            driver.x().onTrue(Commands.either(
            Commands.sequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(40.0)),
            arm.moveArmPositionCommand(19.0),
            arm.moveWristPositionCommand(2.2)
        ),
        Commands.none(),
        () -> demoMode.getBoolean(false)
        ));
        
        // Amp
        operator.y().onTrue(Commands.sequence(
            arm.moveArmPositionCommand(30.25),
            arm.moveWristPositionCommand(-1.2),
            Commands.runOnce(() -> arm.setShooterMotorVelocity(25.0))
        ));

        //Demo Amp
        /*if(demoMode.getBoolean(false)){
            driver.y().onTrue(Commands.sequence(
            arm.moveArmPositionCommand(30.25),
            arm.moveWristPositionCommand(-1.2),
            Commands.runOnce(() -> arm.setShooterMotorVelocity(20.0))
        ));
        }*/

        // Stop shooter motors, reset to ground
        operator.rightTrigger().onTrue(Commands.sequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
            arm.moveWristPositionCommand(0.5),
            arm.moveArmPositionCommand(0.0)
        ));

        //Demo Intake mode
            driver.a().onTrue(Commands.either(
            Commands.sequence(
            Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
            arm.moveWristPositionCommand(0.5),
            arm.moveArmPositionCommand(0.0)
        ),
        Commands.none(),
        () -> demoMode.getBoolean(false)
        ));
        
        // Extend Climber
        operator.rightBumper().onTrue(Commands.runOnce(() -> climber.climbExtend(), climber));
        // Retract Climber
        operator.leftBumper().onTrue(Commands.runOnce(() -> climber.climbRetract(), climber));

        // Set arm position to zero
        armBottomTrigger.onTrue(Commands.runOnce(() -> arm.resetArmMotorPosition(0.0))
            .onlyIf(() -> arm.getArmPosition() < 0.0)
            .ignoringDisable(true));
        
        wristTopTrigger.onTrue(Commands.runOnce(() -> arm.resetWristMotorPosition(arm.getWristAbsPosition()))
            .ignoringDisable(true));
        
        // Set LEDs to Green when Note triggers linebreak
        lineBreakTrigger.onTrue(Commands.runOnce(() -> {
            for (int i = 0; i < ledData.getLength(); i++) {
                ledData.setRGB(i, 0, 255, 0);
            }
            leds.setData(ledData);
        }).ignoringDisable(true));

        // Set LEDs to Red when no note in Line Break
        lineBreakTrigger.onFalse(Commands.runOnce(() -> {
            for (int i = 0; i < ledData.getLength(); i++) {
                ledData.setRGB(i, 255, 0, 0);
            }
            leds.setData(ledData);
        }).ignoringDisable(true));

        NamedCommands.registerCommand("MoveWristToPickup", arm.moveWristPositionCommand(0.5));

        NamedCommands.registerCommand("retractArm", Commands.runOnce(() -> climber.climbRetract(), climber));

        NamedCommands.registerCommand("Shoot", Commands.sequence(
            arm.spinShooterMotorCommand(65.0),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> arm.setIntakePercent(1.0), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                // arm.setShooterMotorVelocity(0.0); // Kevin action
            }, arm)
        ));

        NamedCommands.registerCommand("DropWristAndShoot", Commands.sequence(
            arm.moveWristPositionCommand(0.5),
            arm.spinShooterMotorCommand(65.0),
            Commands.waitSeconds(0.33),
            Commands.runOnce(() -> arm.setIntakePercent(1.0), arm),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                // arm.setShooterMotorVelocity(0.0); // Kevin action
            }, arm)
        ));

        NamedCommands.registerCommand("Intake", Commands.startEnd(
            () -> arm.setIntakePercent(0.4),
            () -> arm.setIntakePercent(0.0),
            arm
        ).until(() -> arm.getLineBreak()));

        NamedCommands.registerCommand("Out-In Nickey", Commands.sequence(
            Commands.runOnce(() -> arm.setIntakePercent(-0.3)),
            Commands.waitSeconds(0.1),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.3), 
                () -> arm.setIntakePercent(0.0))
                .until(() -> arm.getLineBreak())
        ));

        NamedCommands.registerCommand("Clamshell", Commands.sequence(
            Commands.either(
                Commands.runOnce(() -> arm.setShooterMotorVelocity(30.0)),
                Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
                () -> arm.getLineBreak()),
            arm.moveArmPositionCommand(19.0),
            Commands.waitSeconds(0.2).onlyIf(() -> arm.getArmPosition() > 25.0),
            arm.moveWristPositionCommand(-20.5)
        ));
        
        NamedCommands.registerCommand("LiftWrist3.8", arm.moveWristPositionCommand(3.8));
        NamedCommands.registerCommand("LiftWrist3.9", arm.moveWristPositionCommand(3.9));
        NamedCommands.registerCommand("LiftWrist4.0", arm.moveWristPositionCommand(4.0));
        NamedCommands.registerCommand("LiftWrist4.1", arm.moveWristPositionCommand(4.1));
        NamedCommands.registerCommand("LiftWrist4.2", arm.moveWristPositionCommand(4.2));
        NamedCommands.registerCommand("LiftWrist4.3", arm.moveWristPositionCommand(4.3));
        NamedCommands.registerCommand("LiftWrist4.4", arm.moveWristPositionCommand(4.4));
        NamedCommands.registerCommand("LiftWrist4.5", arm.moveWristPositionCommand(4.5));
        NamedCommands.registerCommand("LiftWrist4.8", arm.moveWristPositionCommand(4.8));
        NamedCommands.registerCommand("LiftWrist5.0", arm.moveWristPositionCommand(5.0));
        NamedCommands.registerCommand("LiftWrist5.1", arm.moveWristPositionCommand(5.1));
        NamedCommands.registerCommand("LiftWrist5.2", arm.moveWristPositionCommand(5.2));
        NamedCommands.registerCommand("LiftWrist5.3", arm.moveWristPositionCommand(5.3));
        NamedCommands.registerCommand("LiftWrist5.4", arm.moveWristPositionCommand(5.4));
        NamedCommands.registerCommand("LiftWrist5.5", arm.moveWristPositionCommand(5.5));
        NamedCommands.registerCommand("LiftWrist5.6", arm.moveWristPositionCommand(5.6));
        NamedCommands.registerCommand("LiftWrist5.7", arm.moveWristPositionCommand(5.7));
        NamedCommands.registerCommand("LiftWrist5.8", arm.moveWristPositionCommand(5.8));
        NamedCommands.registerCommand("LiftWrist5.9", arm.moveWristPositionCommand(5.9));
        NamedCommands.registerCommand("LiftWrist6.0", arm.moveWristPositionCommand(6.0));
        NamedCommands.registerCommand("LiftWrist6.1", arm.moveWristPositionCommand(6.1));
        NamedCommands.registerCommand("LiftWrist6.2", arm.moveWristPositionCommand(6.2));
        NamedCommands.registerCommand("LiftWrist6.3", arm.moveWristPositionCommand(6.3));
        NamedCommands.registerCommand("LiftWrist6.4", arm.moveWristPositionCommand(6.4));
        NamedCommands.registerCommand("LiftWrist6.5", arm.moveWristPositionCommand(6.5));
        NamedCommands.registerCommand("LiftWrist6.6", arm.moveWristPositionCommand(6.6));
        NamedCommands.registerCommand("LiftWrist6.7", arm.moveWristPositionCommand(6.7));
        NamedCommands.registerCommand("LiftWrist6.8", arm.moveWristPositionCommand(6.8));
        NamedCommands.registerCommand("LiftWrist6.9", arm.moveWristPositionCommand(6.9));
        NamedCommands.registerCommand("LiftWrist7.0", arm.moveWristPositionCommand(7.0));

        NamedCommands.registerCommand("Stop Wheels", arm.spinShooterMotorCommand(0));

        NamedCommands.registerCommand("Reset Gyro", Commands.runOnce(() -> {
            drivetrain.seedFieldRelative();
        }));

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

        demoMode = Shuffleboard.getTab("Competition")
            .add("Demo Mode", false)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 1)
            .getEntry();

        configureBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        shuffleboard.addAutoChooser(autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setArmCoast() {
        arm.configureMotorsCoast();
    }

    public void setArmBrake() {
        arm.configureMotorsBrake();
    }

    public void retractClimber() {
        Commands.runOnce(() -> climber.climbRetract(), climber);
    }
}
