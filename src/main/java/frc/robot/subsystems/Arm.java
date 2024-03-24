package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

public class Arm extends SubsystemBase {
    // Arm motors
    // Front Right Arm motor (ID 9)
    private final TalonFX armMotor = new TalonFX(9);
    // Back Right Arm motor (ID 2)
    private final TalonFX armMotorFollow = new TalonFX(2);
    // Front Left Arm motor (ID 3)
    private final TalonFX armMotorFollowReverseFront = new TalonFX(3);
    // Back Left Arm motor (ID 4)
    private final TalonFX armMotorFollowReverseBack = new TalonFX(4);
    // Gear ratio between arm motors and arm output
    private final double armGearRatio = (12.0 / 60.0) * (36.0 / 50.0) * (18.0 / 50.0) * (10.0 / 64.0);

    // Wrist motors
    // Wrist Top motor (ID 8)
    private final TalonFX wristMotor = new TalonFX(8);
    // Wrist Bottom motor (ID 7)
    private final TalonFX wristMotorFollow = new TalonFX(7);
    // Gear ratio between wrist motors and wrist output
    private final double wristGearRatio = (12.0 / 84.0) * (18.0 / 84.0) * (24.0 / 64.0); // 1 degree approximately 0.25 rotations

    // Shooter motor (ID 5)
    private final TalonFX shooterMotor = new TalonFX(5);
    // Gear ratio between shooter motor and shooter output
    private final double shooterGearRatio = 1.0;

    // Intake motor (ID 6)
    private final TalonFX intakeMotor = new TalonFX(6);
    
    // Wrist absolute encoder (ID 5)
    private final CANcoder wristEncoder = new CANcoder(5);
    // Offset in degrees added to encoder to make horizontal equal to 0 degrees
    private final double encoderOffset = 73.0;

    // Line Break sensor (RoboRIO Analog channel 3)
    private final AnalogInput lineBreak = new AnalogInput(0);

    // Limit Switch for wrist top (RoboRIO DIO channel 0)
    private final DigitalInput wristTopLimit = new DigitalInput(0);
    // Limit Switch for arm bottom (RoboRIO DIO channel 1)
    private final DigitalInput armBottomLimit = new DigitalInput(1);

    // Motor configs for output
    private final MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    private final MotorOutputConfigs armMotorConfigs = new MotorOutputConfigs();
    private final MotorOutputConfigs motorConfigsReversed = new MotorOutputConfigs();
    private final MotorOutputConfigs armMotorConfigsReversed = new MotorOutputConfigs();

    // Motor configs for current
    private final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
    private final CurrentLimitsConfigs currentLimitDisable = new CurrentLimitsConfigs();

    // Motor configs for PID
    private final Slot0Configs wristPID = new Slot0Configs();
    private final Slot0Configs armPID = new Slot0Configs();
    private final Slot0Configs shooterPID = new Slot0Configs();

    // Encoder configs
    private final MagnetSensorConfigs encoderConfigs = new MagnetSensorConfigs();

    // Motor output requests
    private final DutyCycleOut motorOutputRequest = new DutyCycleOut(0.0);
    private final PositionVoltage motorPositionRequest = new PositionVoltage(0.0).withSlot(0);
    private final VelocityVoltage motorVelocityRequest = new VelocityVoltage(0.0).withSlot(0);

    // Motor signals that store the states of the motors and encoders
    // Note that storing voltages is required to get motor following to work
    private final StatusSignal<Double> armPosition;
    private final StatusSignal<Double> wristPosition;
    private final StatusSignal<Double> wristAbsPosition;
    private final StatusSignal<Double> shooterVelocity;
    private final StatusSignal<Double> armFLVoltage;
    private final StatusSignal<Double> armFRVoltage;
    private final StatusSignal<Double> armBLVoltage;
    private final StatusSignal<Double> armBRVoltage;
    private final StatusSignal<Double> wristTopVoltage;
    private final StatusSignal<Double> wristBottomVoltage;

    // Disable Arm Overrides since they cause loop overruns
    // Uncomment this and 2 other places to reenable
    /*
    private final GenericEntry armPositionOverride;
    private final GenericEntry armPositionOverrideValue;
    private final GenericEntry wristPositionOverride;
    private final GenericEntry wristPositionOverrideValue;
    private final GenericEntry shooterRPMOverride;
    private final GenericEntry shooterRPMOverrideValue;
    */

    // :)
    private final Orchestra music = new Orchestra();

    public Arm() {
        // PID for Wrist
        wristPID.kP = 1.1;
        wristPID.kI = 0.0;
        wristPID.kD = 0.05;

        // PID for arm
        armPID.kP = 1.5;
        armPID.kI = 0.0;
        armPID.kD = 0.1;

        // PID for shooter
        shooterPID.kS = 0.05;
        shooterPID.kV = 0.125;
        shooterPID.kP = 0.3;
        shooterPID.kI = 0.0;
        shooterPID.kD = 0.0;

        // Apply PID values to motors
        armMotor.getConfigurator().apply(armPID);
        armMotorFollow.getConfigurator().apply(armPID);
        armMotorFollowReverseFront.getConfigurator().apply(armPID);
        armMotorFollowReverseBack.getConfigurator().apply(armPID);
        wristMotor.getConfigurator().apply(wristPID);
        wristMotorFollow.getConfigurator().apply(wristPID);
        shooterMotor.getConfigurator().apply(shooterPID);

        // Set wrist encoder as reversed
        encoderConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Apply encoder reversed setting
        wristEncoder.getConfigurator().apply(encoderConfigs);

        // Set 30 Amp current limit setting
        currentLimit.StatorCurrentLimitEnable = true;
        currentLimit.StatorCurrentLimit = 30.0;
        
        // Set disabled current limit setting
        currentLimitDisable.StatorCurrentLimitEnable = false;

        // Apply current limit to arm and wrist motors
        armMotor.getConfigurator().apply(currentLimit);
        armMotorFollow.getConfigurator().apply(currentLimit);
        armMotorFollowReverseFront.getConfigurator().apply(currentLimit);
        armMotorFollowReverseBack.getConfigurator().apply(currentLimit);
        wristMotor.getConfigurator().apply(currentLimit);
        wristMotorFollow.getConfigurator().apply(currentLimit);
        // Disable current limit to intake and shooter
        intakeMotor.getConfigurator().apply(currentLimitDisable);
        shooterMotor.getConfigurator().apply(currentLimitDisable);

        // Set reversed motor setting
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;

        // Set arm and wrist motor settings for max 50% power
        armMotorConfigs.PeakForwardDutyCycle = 0.5;
        armMotorConfigs.PeakReverseDutyCycle = -0.5;

        // Set reversed arm and wrist motor settings for max 50% power
        armMotorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigsReversed.PeakForwardDutyCycle = 0.5;
        armMotorConfigsReversed.PeakReverseDutyCycle = -0.5;

        // Apply arm motor settings to arm
        armMotor.getConfigurator().apply(armMotorConfigs);
        armMotorFollow.getConfigurator().apply(armMotorConfigs);
        // Apply reversed arm motor settings to arm and wrist
        armMotorFollowReverseFront.getConfigurator().apply(armMotorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(armMotorConfigsReversed);
        wristMotor.getConfigurator().apply(armMotorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(armMotorConfigsReversed);
        // Apply reversed motor settings to intake 
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
        // Apply default motor settings to shooter
        shooterMotor.getConfigurator().apply(motorConfigs);

        // Configure all motors to coast initially
        this.configureMotorsCoast();

        // Get all of the signals to read from the motors and encoder
        armPosition = armMotor.getPosition();
        wristAbsPosition = wristEncoder.getAbsolutePosition();
        wristPosition = wristMotor.getPosition();
        shooterVelocity = shooterMotor.getVelocity();
        armFLVoltage = armMotor.getMotorVoltage();
        armFRVoltage = armMotorFollowReverseFront.getMotorVoltage();
        armBLVoltage = armMotorFollow.getMotorVoltage();
        armBRVoltage = armMotorFollowReverseBack.getMotorVoltage();
        wristTopVoltage = wristMotor.getMotorVoltage();
        wristBottomVoltage = wristMotorFollow.getMotorVoltage();

        // Update frequency set to 50 Hz (20 ms)
        BaseStatusSignal.setUpdateFrequencyForAll(50,
            armPosition, wristAbsPosition, wristPosition, shooterVelocity, armFLVoltage, armFRVoltage,
            armBLVoltage, armBRVoltage, wristTopVoltage, wristBottomVoltage);
        // Optimize bus utilization to hide all signals we don't use
        ParentDevice.optimizeBusUtilizationForAll(armMotor, armMotorFollow, armMotorFollowReverseFront, armMotorFollowReverseBack,
            wristMotor, wristMotorFollow, shooterMotor, intakeMotor, wristEncoder);

        // Wait for initial arm position value and initialize to 0 
        armPosition.waitForUpdate(0.02);
        this.resetArmMotorPosition(0.0);

        // Wait for initial wrist absolute position value and initialize wrist to that position
        wristAbsPosition.waitForUpdate(0.02);
        this.resetWristMotorPosition(getWristAbsPosition());

        // Disable Arm Overrides since they cause loop overruns
        // Uncomment this and 2 other places to reenable
        /*
        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armPositionOverride = armTab.add("Arm Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        armPositionOverrideValue = armTab.add("New Arm Position", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 50.0, "block increment", 0.25))
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        wristPositionOverride = armTab.add("Wrist Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();
        wristPositionOverrideValue = armTab.add("New Wrist Position", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -20.0, "max", 30.0, "block increment", 0.1))
            .withPosition(3, 1)
            .withSize(2, 1)
            .getEntry();
        shooterRPMOverride = armTab.add("Shooter Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(6, 0)
            .withSize(2, 1)
            .getEntry();
        shooterRPMOverrideValue = armTab.add("New Shooter RPS", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 100.0, "block increment", 1.0))
            .withPosition(6, 1)
            .withSize(2, 1)
            .getEntry();
        */

        // :)
        music.loadMusic("Kevins Great File.chrp");
        music.addInstrument(intakeMotor);
    }

    // Return arm position in rotations
    public double getArmPosition() {
        return armPosition.getValue();
    }

    // Return arm position in degrees
    public double getArmAngle() {
        return Units.rotationsToDegrees(getArmPosition() * armGearRatio);
    }

    // Return wrist absolute position in rotations
    public double getWristAbsPosition() {
        return wristAbsPosition.getValue() + Units.degreesToRotations(encoderOffset);
    }

    // Return wrist absolute position in degrees
    public double getWristAbsAngle() {
        return Units.rotationsToDegrees(getWristAbsPosition());
    }

    // Return wrist relative position in rotations
    public double getWristPosition() {
        return wristPosition.getValue();
    }

    // Return wrist relative postion in degrees
    public double getWristAngle() {
        return Units.rotationsToDegrees(getWristPosition() * wristGearRatio);
    }

    // Return shooter velocity in rotations per second
    public double getShooterMotorVelocity() {
        return shooterVelocity.getValue();
    }

    // Return shooter velocity in RPM
    public double getShooterWheelRPM() {
        return getShooterMotorVelocity() * shooterGearRatio * 60.0;
    }

    // Return whether the line break is triggered
    public boolean getLineBreak() {
        if (lineBreak.getVoltage() > 3.0) {
            return true;
        } else {
            return false;
        }
    }

    // Return whether the wrist top limit switch is triggered
    public boolean getWristTopLimit() {
        return wristTopLimit.get();
    }

    // Return whether the arm bottom limit switch is triggered
    public boolean getArmBottomLimit() {
        return armBottomLimit.get();
    }

    // Set the arm to a given position in rotations
    public void setArmPosition(double position) {
        // Set the three arm follower motors to follow the lead arm motor
        armMotorFollow.setControl(new StrictFollower(armMotor.getDeviceID()));
        armMotorFollowReverseFront.setControl(new StrictFollower(armMotor.getDeviceID()));
        armMotorFollowReverseBack.setControl(new StrictFollower(armMotor.getDeviceID()));

        // Set the arm motor to the given position
        armMotor.setControl(motorPositionRequest.withPosition(position));
    }

    // Set the arm to a given angle in degrees
    public void setArmAngle(double angle) {
        // Convert the given angle to rotations and set arm to that position
        double position = Units.degreesToRotations(angle) / armGearRatio;
        setArmPosition(position);
    }

    // Set the wrist to a given position in rotations
    public void setWristPosition(double position) {
        // Set the wrist follower motor to follow the lead wrist motor
        wristMotorFollow.setControl(new StrictFollower(wristMotor.getDeviceID()));

        // Set the wrist motor to the given position
        wristMotor.setControl(motorPositionRequest.withPosition(position));
    }

    // Set the wrist to a given angle in degrees
    public void setWristAngle(double angle) {
        // Convert the given angle to rotations and set wrist to that position
        double position = Units.degreesToRotations(angle) / wristGearRatio;
        setWristPosition(position);
    }

    // Set the intake to a given percent power from -1.0 to 1.0
    public void setIntakePercent(double percent) {
        // Set the intake motor to the given percent
        intakeMotor.setControl(motorOutputRequest.withOutput(percent));
    }

    // Set the shooter to a given motor velocity in rotations per second
    public void setShooterMotorVelocity(double velocity) {
        // Set the shooter motor to the given velocity
        shooterMotor.setControl(motorVelocityRequest.withVelocity(velocity));
    }

    // Set the shooter to a given wheel velocity in RPM
    public void setShooterWheelVelocity(double wheelVelocity) {
        // Convert the given RPM to rotations per second and set shooter to that velocity
        double motorVelocity = (wheelVelocity / 60.0) / shooterGearRatio;
        setShooterMotorVelocity(motorVelocity);
    }

    // Configure all motors to coast when not running
    public void configureMotorsCoast() {
        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        armMotorConfigsReversed.NeutralMode = NeutralModeValue.Coast;

        armMotor.getConfigurator().apply(armMotorConfigs);
        armMotorFollow.getConfigurator().apply(armMotorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(armMotorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(armMotorConfigsReversed);
        wristMotor.getConfigurator().apply(armMotorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(armMotorConfigsReversed);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
    }

    // Configure all motors to brake when not running
    public void configureMotorsBrake() {
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        armMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Brake;
        armMotorConfigsReversed.NeutralMode = NeutralModeValue.Brake;

        armMotor.getConfigurator().apply(armMotorConfigs);
        armMotorFollow.getConfigurator().apply(armMotorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(armMotorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(armMotorConfigsReversed);
        wristMotor.getConfigurator().apply(armMotorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(armMotorConfigsReversed);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
    }

    // Arm Command that moves the arm to the given position
    public Command moveArmPositionCommand(double position)
    {
        // Run the set arm position function continuously
        return this.run(() -> this.setArmPosition(position))
            // Move on to the next command when the arm gets "close enough" to the given position
            .until(() -> {
                // Calculate the difference between the current position and the target position
                double positionDiff = Math.abs(this.getArmPosition() - position);
                // Finish this command when the position difference is less than 19.0 rotations
                // This number is very large so that the next command starts sooner
                // This allows the arm and wrist to move together
                return positionDiff < 19.0;
            });
    }

    // Arm Command that moves the arm to the given angle
    public Command moveArmAngleCommand(double angle)
    {
        // Run the set arm angle function continuously
        return this.run(() -> this.setArmAngle(angle))
            // Move on to the next command when the arm gets "close enough" to the given angle
            .until(() -> {
                // Calculate the difference between the current angle and the target angle
                double angleDiff = Math.abs(this.getArmAngle() - angle);
                // Finish this command when the angle difference is less than 55.0 degrees
                // This number is very large so that the next command starts sooner
                // This allows the arm and wrist to move together
                return angleDiff < 55.0;
            });
    }

    // Arm Command that moves the wrist to the given positions
    public Command moveWristPositionCommand(double position)
    {
        // Run the set wrist position function continuously
        return this.run(() -> this.setWristPosition(position))
            // Move on to the next command when the wrist gets "close enough" to the given position
            .until(() -> {
                // Calculate the difference between the current position and the target position
                double positionDiff = Math.abs(this.getWristPosition() - position);
                // Finish this command when the position difference is less than 23.0 rotations
                // This number is very large so that the next command starts sooner
                // This allows the arm and wrist to move together
                return positionDiff < 23.0;
            });
    }

    // Arm Command that moves the wrist to the given angle
    public Command moveWristAngleCommand(double angle)
    {
        // Run the set wrist angle function continuously
        return this.run(() -> this.setWristAngle(angle))
            // Move on to the next command when the wrist gets "close enough" to the given angle
            .until(() -> {
                // Calculate the difference between the current angle and the target angle
                double angleDiff = Math.abs(this.getWristAngle() - angle);
                // Finish this command when the angle difference is less than 95.0 degrees
                // This number is very large so that the next command starts sooner
                // This allows the arm and wrist to move together
                return angleDiff < 95.0;
            });
    }

    // Arm Command that spins the shooter motor at the given velocity in rotations per second
    public Command spinShooterMotorCommand(double velocity) {
        // Run the set shooter motor velocity function continuously
        return this.run(() -> this.setShooterMotorVelocity(velocity))
            // Move on to the next command when the shooter gets "close enough" to the given velocity
            .until(() -> {
                // Calculate the difference between the current velocity and the target velocity
                double speedDiff = Math.abs(this.getShooterMotorVelocity() - velocity);
                // Finish this command when the velocity difference is less than 1.0 rotations per second
                // This number makes sure that the shooter is spinning at the correct velocity to shoot
                return speedDiff < 1.0;
            });
    }

    // Arm Command that spins the shooter motor at the given wheel velocity in RPM
    public Command spinShooterWheelCommand(double wheelVelocity) {
        // Run the set shooter motor wheel velocity function continuously
        return this.run(() -> this.setShooterWheelVelocity(wheelVelocity))
            // Move on to the next command when the shooter gets "close enough" to the given wheel velocity
            .until(() -> {
                // Calculate the difference between the current wheel velocity and the target wheel velocity
                double speedDiff = Math.abs(this.getShooterWheelRPM() - wheelVelocity);
                // Finish this command when the velocity difference is less than 60.0 RPM
                // This number makes sure that the shooter is spinning at the correct velocity to shoot
                return speedDiff < 60.0;
            });
    }

    // Reset the arm motor to the given position
    public void resetArmMotorPosition(double newPosition) {
        // Convert the given position to a motor position
        double motorPosition = newPosition / armGearRatio;
        // Apply the motor position to all four arm motors
        armMotor.setPosition(motorPosition);
        armMotorFollow.setPosition(motorPosition);
        armMotorFollowReverseFront.setPosition(motorPosition);
        armMotorFollowReverseBack.setPosition(motorPosition);
    }

    // Reset the wrist motor to the given position
    public void resetWristMotorPosition(double newPosition) {
        // Convert the given position to a motor position
        double motorPosition = newPosition / wristGearRatio;
        // Apply the motor position to both wrist motors
        wristMotor.setPosition(motorPosition);
        wristMotorFollow.setPosition(motorPosition);
    }

    // :)
    public void playMusic() {
        music.play();
    }

    // :)
    public void stopMusic() {
        music.stop();
    }

    // Periodic function that runs once per robot loop
    @Override
    public void periodic() {
        // Refresh all motor and encoder signals
        BaseStatusSignal.refreshAll(armPosition, wristAbsPosition, wristPosition, shooterVelocity, armFLVoltage, armFRVoltage,
            armBLVoltage, armBRVoltage, wristTopVoltage, wristBottomVoltage);

        // Reset the wrist motor position using the absolute encoder
        this.resetWristMotorPosition(this.getWristAbsPosition());

        // Disable Arm Overrides since they cause loop overruns
        // Uncomment this and 2 other places to reenable

        /*
        if (armPositionOverride.getBoolean(false)) {
            armMotorFollow.setControl(new StrictFollower(armMotor.getDeviceID())); // Back right follows Front right
            armMotorFollowReverseFront.setControl(new StrictFollower(armMotor.getDeviceID())); // Front left follows and opposes Front Right
            armMotorFollowReverseBack.setControl(new StrictFollower(armMotor.getDeviceID())); // Back left follows and opposes Front Right

            double armPosition = armPositionOverrideValue.getDouble(0.0);
            armMotor.setControl(motorPositionRequest.withPosition(armPosition));
        }

        if (wristPositionOverride.getBoolean(false)) {
            wristMotorFollow.setControl(new StrictFollower(wristMotor.getDeviceID())); // Bottom wrist follows top

            double wristPosition = wristPositionOverrideValue.getDouble(0.0);
            wristMotor.setControl(motorPositionRequest.withPosition(wristPosition));
        }

        if (shooterRPMOverride.getBoolean(false)) {
            double shooterRPM = shooterRPMOverrideValue.getDouble(0.0);
            shooterMotor.setControl(motorVelocityRequest.withVelocity(shooterRPM));
        }
        */
    }
}
