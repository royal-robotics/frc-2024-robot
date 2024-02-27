// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

    private final PhotonCamera aprilTag = new PhotonCamera("AprilTag");
    private final Arm arm = new Arm(aprilTag); // Arm subsystem
    private final Climber climber = new Climber(); // Climber subsystem
    private final OurShuffleboard shuffleboard = new OurShuffleboard(drivetrain, arm);
    private final AddressableLED leds = new AddressableLED(0);
    private final AddressableLEDBuffer ledData = new AddressableLEDBuffer(12);

    private final Trigger armBottomTrigger = new Trigger(() -> arm.getArmBottomLimit());
    private final Trigger wristTopTrigger = new Trigger(() -> arm.getWristTopLimit());
    private final Trigger lineBreakTrigger = new Trigger(() -> arm.getLineBreak());

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // I want field-centric
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // driving in open loop

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    private boolean trackingTag = false;
    private double angleFromTag = 0.0;

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        // reset the field-centric heading on left bumper press
        driver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        driver.leftBumper().whileTrue(Commands.startEnd(
          () -> {
            MaxSpeed = 2.5;
            MaxAngularRate = 1.0 * Math.PI;
          }, 
          () -> {
            MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
            MaxAngularRate = 3.0 * Math.PI;
          }
        ));
        driver.rightBumper().whileTrue(Commands.either(
            Commands.startEnd(
                () -> arm.setIntakePercent(0.6),
                () -> arm.setIntakePercent(0.0)),
            Commands.startEnd(
                () -> arm.setIntakePercent(0.5),
                () -> arm.setIntakePercent(0.0))
                .until(() -> arm.getLineBreak()),
            () -> arm.getLineBreak()));
        /*driver.rightTrigger().whileTrue(Commands.either(
            Commands.startEnd(
                () -> arm.setShooterMotorVelocity(45.0),
                () -> arm.setShooterMotorVelocity(0.0)),
            Commands.startEnd(
                () -> arm.setShooterMotorVelocity(75.0),
                () -> arm.setShooterMotorVelocity(0.0)),
            () -> arm.getArmPosition() >= 10));*/
        driver.rightTrigger().onTrue(Commands.sequence(
            arm.spinWheelsCommand(70.0),
            Commands.runOnce(() -> arm.setIntakePercent(0.5)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0);
            })
        ));
        driver.y().whileTrue(Commands.startEnd(() -> arm.setIntakePercent(-0.6), () -> arm.setIntakePercent(0.0)));
        driver.a().onTrue(Commands.sequence(
          Commands.runOnce(() -> arm.setShooterMotorVelocity(52)),
          arm.moveArmPositionCommand(19.0),
          arm.moveWristPositionCommand(-8.55),
            drivetrain.applyRequest(() -> {
                if (arm.hasAprilTag()) {
                    double angleToDrive = 0.0;
                    List<PhotonTrackedTarget> targets = arm.getAprilTags();
                    for (PhotonTrackedTarget target : targets) {
                        int id = target.getFiducialId();
                        if (id == 4 || id == 7) {
                            for (int i = 0; i < ledData.getLength(); i++) {
                                ledData.setRGB(i, 0, 128, 0);
                            }
                            leds.setData(ledData);
                            this.trackingTag = true;
                            this.angleFromTag = target.getYaw() - 5.0;
                            angleToDrive = -Units.degreesToRadians(this.angleFromTag) * MaxAngularRate * 1.2;
                            break;
                        } else {
                            for (int i = 0; i < ledData.getLength(); i++) {
                                ledData.setRGB(i, 128, 128, 0);
                            }
                            leds.setData(ledData);
                            this.trackingTag = false;
                            this.angleFromTag = 0.0;
                            angleToDrive = Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate;
                        }
                    }
                    System.out.println(angleToDrive);
                    return drive
                        .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(angleToDrive); // Drive counterclockwise with negative X (left)
                } else {
                    for (int i = 0; i < ledData.getLength(); i++) {
                        ledData.setRGB(i, 0, 0, 128);
                    }
                    leds.setData(ledData);
                    return drive
                        .withVelocityX(Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate); // Drive counterclockwise with negative X (left)
                }
            }
            /*Commands.run(() -> {
                if (arm.hasAprilTag()) {
                    List<PhotonTrackedTarget> targets = arm.getAprilTags();
                    for (PhotonTrackedTarget target : targets) {
                        int id = target.getFiducialId();
                        if (id == 4 || id == 7) {
                            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                                Units.inchesToMeters(32.0), 
                                Units.inchesToMeters(57.375), 
                                Units.degreesToRadians(35.0), 
                                Units.degreesToRadians(target.getPitch()))
                                * 2 - 1; // Distance off
                            arm.setWristPosition(1.7 * distance - 14.35); // Regression
                        }
                        break;
                    }
                }
            })*/
          ),
          /*Commands.run(() -> {
            if (arm.hasAprilTag()) {
                double angleSpeed = 0.0;
                List<PhotonTrackedTarget> targets = arm.getAprilTags();
                for (PhotonTrackedTarget target : targets) {
                    int id = target.getFiducialId();
                    if (id == 4 || id == 7) { 
                        for (int i = 0; i < ledData.getLength(); i++) {
                            ledData.setRGB(i, 0, 128, 0);
                        }
                        leds.setData(ledData);
                        this.trackingTag = true;
                        this.angleFromTag = target.getYaw();
                        angleSpeed = angleFromTag;
                    } else {
                        for (int i = 0; i < ledData.getLength(); i++) {
                            ledData.setRGB(i, 128, 128, 0);
                        }
                        leds.setData(ledData);
                        this.trackingTag = false;
                        this.angleFromTag = 0.0;
                        angleSpeed = Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate;
                    }
                }
                System.out.println(angleSpeed);
            } else {
                for (int i = 0; i < ledData.getLength(); i++) {
                    ledData.setRGB(i, 0, 0, 128);
                }
                leds.setData(ledData);
            }
            drivetrain.setChassisSpeeds(new ChassisSpeeds(
                Math.copySign(Math.pow(-driver.getLeftY(), 2), -driver.getLeftY()) * MaxSpeed,
                Math.copySign(Math.pow(-driver.getLeftX(), 2), -driver.getLeftX()) * MaxSpeed,
                Math.copySign(Math.pow(-driver.getRightX(), 2), -driver.getRightX()) * MaxAngularRate
            ));
          }, drivetrain).withTimeout(30.0),*/
          Commands.runOnce(() -> {
                for (int i = 0; i < ledData.getLength(); i++) {
                    ledData.setRGB(i, 128, 0, 0);
                }
                leds.setData(ledData);
          })
        ));

        operator.leftTrigger().onTrue(Commands.sequence(
          Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
          arm.moveArmPositionCommand(19.0),
          arm.moveWristPositionCommand(-20.5)
        ));
        operator.y().onTrue(Commands.sequence(
          arm.moveArmPositionCommand(30.25),
          arm.moveWristPositionCommand(-1.0),
          arm.spinWheelsCommand(45.0)
        ));
        operator.rightTrigger().onTrue(Commands.sequence(
          Commands.runOnce(() -> arm.setShooterMotorVelocity(0.0)),
          arm.moveWristPositionCommand(0.5),
          arm.moveArmPositionCommand(0.0)
        ));
        operator.rightBumper().onTrue(Commands.runOnce(() -> climber.climbExtend(), climber));
        operator.leftBumper().onTrue(Commands.runOnce(() -> climber.climbRetract(), climber));

        armBottomTrigger.onTrue(Commands.runOnce(() -> arm.resetArmMotorPosition(0.0))
            .ignoringDisable(true));
        wristTopTrigger.onTrue(Commands.runOnce(() -> arm.resetWristMotorPosition(arm.getWristAbsPosition()))
            .ignoringDisable(true));
        lineBreakTrigger.onTrue(Commands.runOnce(() -> {
            for (int i = 0; i < ledData.getLength(); i++) {
                ledData.setRGB(i, 0, 128, 0);
            }
            leds.setData(ledData);
        }).ignoringDisable(true));
        lineBreakTrigger.onFalse(Commands.runOnce(() -> {
            for (int i = 0; i < ledData.getLength(); i++) {
                ledData.setRGB(i, 128, 0, 0);
            }
            leds.setData(ledData);
        }).ignoringDisable(true));

        NamedCommands.registerCommand("MoveWristToPickup", arm.moveWristPositionCommand(0.5));

        NamedCommands.registerCommand("Shoot", Commands.sequence(
            arm.spinWheelsCommand(90.0),
            Commands.runOnce(() -> arm.setIntakePercent(0.6), arm),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
                arm.setIntakePercent(0.0);
                arm.setShooterMotorVelocity(0.0); // Kevin action
            }, arm)
        ));

        NamedCommands.registerCommand("Intake", Commands.startEnd(
            () -> arm.setIntakePercent(0.5),
            () -> arm.setIntakePercent(0.0),
            arm
        ).until(() -> arm.getLineBreak()));

        NamedCommands.registerCommand("LiftWrist4.6", arm.moveWristPositionCommand(4.6));
        NamedCommands.registerCommand("LiftWrist3.5", arm.moveWristPositionCommand(3.5));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        leds.setLength(ledData.getLength());
        for (int i = 0; i < ledData.getLength(); i++) {
            ledData.setRGB(i, 128, 0, 0);
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
}
