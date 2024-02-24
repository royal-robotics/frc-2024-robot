package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(9); // Front Right ID 9
    private final TalonFX armMotorFollow = new TalonFX(2); // Back Right ID 2
    private final TalonFX armMotorFollowReverseFront = new TalonFX(3); // Front Left ID 3
    private final TalonFX armMotorFollowReverseBack = new TalonFX(4); // Back Left ID 4
    private final double armGearRatio = (12.0 / 60.0) * (36.0 / 50.0) * (18.0 / 50.0) * (10.0 / 64.0);

    private final TalonFX wristMotor = new TalonFX(8); // Wrist top ID 8
    private final TalonFX wristMotorFollow = new TalonFX(7); // wrist Bottom ID 7
    private final double wristGearRatio = (12.0 / 84.0) * (18.0 / 84.0) * (24.0 / 64.0);

    private final TalonFX shooterMotor = new TalonFX(5); // Shooter motor ID 5
    private final double shooterGearRatio = 1.0;

    private final TalonFX intakeMotor = new TalonFX(6); // Intake motor ID 6
    
    private final CANcoder armEncoder = new CANcoder(5); 
    private final double encoderOffset = 17.5;

    private final StrictFollower followArm = new StrictFollower(9); // Front right as main motor
    
    private final StrictFollower followWrist = new StrictFollower(8); // Wrist top is to be folowed

    private final MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    private final MotorOutputConfigs motorConfigsReversed = new MotorOutputConfigs();

    private final MagnetSensorConfigs encoderConfigs = new MagnetSensorConfigs();

    private final DutyCycleOut motorOutputRequest = new DutyCycleOut(0.0);
    private final PositionVoltage motorPositionRequest = new PositionVoltage(0.0).withSlot(0);
    private final VelocityVoltage motorVelocityRequest = new VelocityVoltage(0.0).withSlot(0);

    private final Slot0Configs wristPID = new Slot0Configs(); // For wrist
    private final Slot0Configs armPID = new Slot0Configs(); // For arm
    private final Slot0Configs shooterPID = new Slot0Configs(); // For shooter

    private final AnalogInput lineBreak = new AnalogInput(3); // Linebreak Sensor on channel 3

    private final DigitalInput wristTopLimit = new DigitalInput(0); // Wrist limit switch at top
    private final DigitalInput armBottomLimit = new DigitalInput(1); // Arm limit switch for arm at 0 on channel 1

    private final StatusSignal<Double> armPosition;
    private final StatusSignal<Double> wristPosition;
    private final StatusSignal<Double> wristAbsPosition;
    private final StatusSignal<Double> shooterVelocity;
    private final StatusSignal<Double> wristTopVoltage;
    private final StatusSignal<Double> wristBottomVoltage;

    private final GenericEntry armPositionOverride;
    private final GenericEntry armPositionOverrideValue;
    private final GenericEntry wristPositionOverride;
    private final GenericEntry wristPositionOverrideValue;

    private final Orchestra music = new Orchestra();
    public Arm() {
        // PID for Wrist
        wristPID.kP = 1.1;
        wristPID.kI = 0.0;
        wristPID.kD = 0.05;

        // PID for arm
        armPID.kP = 2.0;
        armPID.kI = 0.0;
        armPID.kD = 0.2;

        // PID for shooter
        shooterPID.kS = 0.05;
        shooterPID.kV = 0.1;
        shooterPID.kP = 0.0;
        shooterPID.kI = 0.0;
        shooterPID.kD = 0.0;

        armMotor.getConfigurator().apply(armPID);
        armMotorFollow.getConfigurator().apply(armPID);
        armMotorFollowReverseFront.getConfigurator().apply(armPID);
        armMotorFollowReverseBack.getConfigurator().apply(armPID);
        wristMotor.getConfigurator().apply(wristPID);
        wristMotorFollow.getConfigurator().apply(wristPID);
        shooterMotor.getConfigurator().apply(shooterPID);

        encoderConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        armEncoder.getConfigurator().apply(encoderConfigs);

        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;

        armMotor.getConfigurator().apply(motorConfigs);
        armMotorFollow.getConfigurator().apply(motorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(motorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(motorConfigsReversed);
        wristMotor.getConfigurator().apply(motorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(motorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);

        armPosition = armMotor.getPosition();
        wristAbsPosition = armEncoder.getAbsolutePosition();
        wristPosition = wristMotor.getPosition();
        shooterVelocity = shooterMotor.getVelocity();
        wristTopVoltage = wristMotor.getMotorVoltage();
        wristBottomVoltage = wristMotorFollow.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            armPosition, wristAbsPosition, wristPosition, shooterVelocity, wristTopVoltage, wristBottomVoltage);
        ParentDevice.optimizeBusUtilizationForAll(armMotor, armMotorFollow, armMotorFollowReverseFront, armMotorFollowReverseBack,
            wristMotor, wristMotorFollow, shooterMotor, intakeMotor);

        armPosition.waitForUpdate(0.02);
        this.resetArmMotorPosition(0.0);

        wristAbsPosition.waitForUpdate(0.02);
        this.resetWristMotorPosition(getWristAbsPosition());

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        armPositionOverride = armTab.add("Arm Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        armPositionOverrideValue = armTab.add("New Arm Position", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 50.0, "block increment", 1.0))
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
            .withProperties(Map.of("min", -20.0, "max", 30.0, "block increment", 1.0))
            .withPosition(3, 1)
            .withSize(2, 1)
            .getEntry();

        music.loadMusic("test.chrp");
        music.addInstrument(intakeMotor);
    }

    public double getArmPosition() {
        return armPosition.getValue();
    }

    public double getArmAngle() {
        return Units.rotationsToDegrees(getArmPosition() * armGearRatio);
    }

    public double getWristAbsPosition() {
        return wristAbsPosition.getValue() + Units.degreesToRotations(encoderOffset);
    }

    public double getWristAbsAngle() {
        return Units.rotationsToDegrees(getWristAbsPosition());
    }

    public double getWristPosition() {
        return wristPosition.getValue();
    }

    public double getWristAngle() {
        return Units.rotationsToDegrees(getWristPosition() * wristGearRatio);
    }

    public double getShooterMotorVelocity() {
        return shooterVelocity.getValue();
    }

    public double getShooterWheelVelocity() {
        return getShooterMotorVelocity() * shooterGearRatio;
    }

    public double getWristTopVoltage() {
        return wristTopVoltage.getValue();
    }

    public double getWristBottomVoltage() {
        return wristBottomVoltage.getValue();
    }

    public boolean getLineBreak() {
        if (lineBreak.getVoltage() > 3) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getWristTopLimit() {
        return wristTopLimit.get();
    }

    public boolean getArmBottomLimit() {
        return armBottomLimit.get();
    }

    public void setArmPosition(double position) {
        armMotor.setControl(motorPositionRequest.withPosition(position));
        armMotorFollow.setControl(followArm); // Back right follows Front right
        armMotorFollowReverseFront.setControl(followArm); // Front left follows and opposes Front Right
        armMotorFollowReverseBack.setControl(followArm); // Back left follows and opposes Front Right
    }

    public void setArmAngle(double angle) {
        double position = Units.degreesToRotations(angle) / armGearRatio;
        setArmPosition(position);
    }

    public void setWristPosition(double position) {
        wristMotor.setControl(motorPositionRequest.withPosition(position));
        wristMotorFollow.setControl(followWrist);
    }

    public void setWristAngle(double angle) {
        double position = Units.degreesToRotations(angle) / wristGearRatio;
        setWristPosition(position);
    }

    public void setIntakePercent(double percent) {
        intakeMotor.setControl(motorOutputRequest.withOutput(percent));
    }

    public void setShooterMotorVelocity(double velocity) {
        shooterMotor.setControl(motorVelocityRequest.withVelocity(velocity));
    }

    public void setShooterWheelVelocity(double wheelVelocity) {
        double motorVelocity = wheelVelocity / shooterGearRatio;
        setShooterMotorVelocity(motorVelocity);
    }

    public void setMotorBrake() { 
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;

        armMotor.getConfigurator().apply(motorConfigs);
        armMotorFollow.getConfigurator().apply(motorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(motorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(motorConfigsReversed);

        wristMotor.getConfigurator().apply(motorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(motorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
    }

    public void setMotorCoast() {
        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;

        armMotor.getConfigurator().apply(motorConfigs);
        armMotorFollow.getConfigurator().apply(motorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(motorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(motorConfigsReversed);

        wristMotor.getConfigurator().apply(motorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(motorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
    }

    public void resetWristMotorPosition(double newPosition) {
        double motorPosition = newPosition / wristGearRatio;
        wristMotor.setPosition(motorPosition);
        wristMotorFollow.setPosition(motorPosition);
    }

    public void resetArmMotorPosition(double newPosition) {
        double motorPosition = newPosition / armGearRatio;
        armMotorFollowReverseFront.setPosition(motorPosition);
        armMotorFollow.setPosition(motorPosition); // Back right follows Front right
        armMotor.setPosition(motorPosition);// Front left follows and opposes Front Right
        armMotorFollowReverseBack.setPosition(motorPosition); // Back left follows and opposes Front Right
    }

    public void playMusic() {
        music.play();
    }

    public void stopMusic() {
        music.stop();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armPosition, wristAbsPosition, wristPosition, shooterVelocity, wristTopVoltage, wristBottomVoltage);

        if (armPositionOverride.getBoolean(false)) {
            double armPosition = armPositionOverrideValue.getDouble(0.0);
            armMotor.setControl(motorPositionRequest.withPosition(armPosition));
            armMotorFollow.setControl(followArm); // Back right follows Front right
            armMotorFollowReverseFront.setControl(followArm); // Front left follows and opposes Front Right
            armMotorFollowReverseBack.setControl(followArm); // Back left follows and opposes Front Right
        }

        if (wristPositionOverride.getBoolean(false)) {
            double wristPosition = wristPositionOverrideValue.getDouble(0.0);
            wristMotor.setControl(motorPositionRequest.withPosition(wristPosition));
            wristMotorFollow.setControl(followWrist); // Bottom wrist follows top
        }
    }
}
