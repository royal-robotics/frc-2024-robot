package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

    private final MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    private final MotorOutputConfigs armMotorConfigs = new MotorOutputConfigs();
    private final MotorOutputConfigs motorConfigsReversed = new MotorOutputConfigs();
    private final MotorOutputConfigs armMotorConfigsReversed = new MotorOutputConfigs();

    private final CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(30.0);
    private final CurrentLimitsConfigs currentLimitDisable = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false);

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
    private final StatusSignal<Double> armFLVoltage;
    private final StatusSignal<Double> armFRVoltage;
    private final StatusSignal<Double> armBLVoltage;
    private final StatusSignal<Double> armBRVoltage;
    private final StatusSignal<Double> wristTopVoltage;
    private final StatusSignal<Double> wristBottomVoltage;

    private final GenericEntry armPositionOverride;
    private final GenericEntry armPositionOverrideValue;
    private final GenericEntry wristPositionOverride;
    private final GenericEntry wristPositionOverrideValue;
    private final GenericEntry shooterRPMOverride;
    private final GenericEntry shooterRPMOverrideValue;

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

        armMotor.getConfigurator().apply(armPID);
        armMotorFollow.getConfigurator().apply(armPID);
        armMotorFollowReverseFront.getConfigurator().apply(armPID);
        armMotorFollowReverseBack.getConfigurator().apply(armPID);
        wristMotor.getConfigurator().apply(wristPID);
        wristMotorFollow.getConfigurator().apply(wristPID);
        shooterMotor.getConfigurator().apply(shooterPID);

        encoderConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        armEncoder.getConfigurator().apply(encoderConfigs);

        armMotor.getConfigurator().apply(currentLimit);
        armMotorFollow.getConfigurator().apply(currentLimit);
        armMotorFollowReverseFront.getConfigurator().apply(currentLimit);
        armMotorFollowReverseBack.getConfigurator().apply(currentLimit);
        wristMotor.getConfigurator().apply(currentLimit);
        wristMotorFollow.getConfigurator().apply(currentLimit);
        intakeMotor.getConfigurator().apply(currentLimitDisable);
        shooterMotor.getConfigurator().apply(currentLimitDisable);

        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        armMotorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigs.PeakForwardDutyCycle = 0.5;
        armMotorConfigs.PeakReverseDutyCycle = -0.5;
        armMotorConfigsReversed.PeakForwardDutyCycle = 0.5;
        armMotorConfigsReversed.PeakReverseDutyCycle = -0.5;

        armMotor.getConfigurator().apply(armMotorConfigs);
        armMotorFollow.getConfigurator().apply(armMotorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(armMotorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(armMotorConfigsReversed);
        wristMotor.getConfigurator().apply(armMotorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(armMotorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);

        armPosition = armMotor.getPosition();
        wristAbsPosition = armEncoder.getAbsolutePosition();
        wristPosition = wristMotor.getPosition();
        shooterVelocity = shooterMotor.getVelocity();
        armFLVoltage = armMotor.getMotorVoltage();
        armFRVoltage = armMotorFollowReverseFront.getMotorVoltage();
        armBLVoltage = armMotorFollow.getMotorVoltage();
        armBRVoltage = armMotorFollowReverseBack.getMotorVoltage();
        wristTopVoltage = wristMotor.getMotorVoltage();
        wristBottomVoltage = wristMotorFollow.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            armPosition, wristAbsPosition, wristPosition, shooterVelocity, armFLVoltage, armFRVoltage,
            armBLVoltage, armBRVoltage, wristTopVoltage, wristBottomVoltage);
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

    public double getShooterWheelRPM() {
        return getShooterMotorVelocity() * shooterGearRatio * 60.0;
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
        armMotorFollow.setControl(new StrictFollower(armMotor.getDeviceID())); // Back right follows Front right
        armMotorFollowReverseFront.setControl(new StrictFollower(armMotor.getDeviceID())); // Front left follows and opposes Front Right
        armMotorFollowReverseBack.setControl(new StrictFollower(armMotor.getDeviceID())); // Back left follows and opposes Front Right

        armMotor.setControl(motorPositionRequest.withPosition(position));
    }

    public void setArmAngle(double angle) {
        double position = Units.degreesToRotations(angle) / armGearRatio;
        setArmPosition(position);
    }

    public void setWristPosition(double position) {
        wristMotorFollow.setControl(new StrictFollower(wristMotor.getDeviceID()));

        wristMotor.setControl(motorPositionRequest.withPosition(position));
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
        armMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Brake;
        armMotorConfigsReversed.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigs.PeakForwardDutyCycle = 0.5;
        armMotorConfigs.PeakReverseDutyCycle = -0.5;
        armMotorConfigsReversed.PeakForwardDutyCycle = 0.5;
        armMotorConfigsReversed.PeakReverseDutyCycle = -0.5;

        armMotor.getConfigurator().apply(armMotorConfigs);
        armMotorFollow.getConfigurator().apply(armMotorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(armMotorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(armMotorConfigsReversed);
        wristMotor.getConfigurator().apply(armMotorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(armMotorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
    }

    public void setMotorCoast() {
        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        armMotorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        armMotorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;
        armMotorConfigs.PeakForwardDutyCycle = 0.5;
        armMotorConfigs.PeakReverseDutyCycle = -0.5;
        armMotorConfigsReversed.PeakForwardDutyCycle = 0.5;
        armMotorConfigsReversed.PeakReverseDutyCycle = -0.5;

        armMotor.getConfigurator().apply(armMotorConfigs);
        armMotorFollow.getConfigurator().apply(armMotorConfigs);
        armMotorFollowReverseFront.getConfigurator().apply(armMotorConfigsReversed);
        armMotorFollowReverseBack.getConfigurator().apply(armMotorConfigsReversed);
        wristMotor.getConfigurator().apply(armMotorConfigsReversed);
        wristMotorFollow.getConfigurator().apply(armMotorConfigsReversed);
        shooterMotor.getConfigurator().apply(motorConfigs);
        intakeMotor.getConfigurator().apply(motorConfigsReversed);
    }

    public Command moveArmPositionCommand(double position)
    {
        return this.run(() -> this.setArmPosition(position))
            .until(() -> {
                double positionDiff = Math.abs(this.getArmPosition() - position);
                return positionDiff < 19.0;
            });
    }

    public Command moveWristPositionCommand(double position)
    {
        return this.run(() -> this.setWristPosition(position))
            .until(() -> {
                double positionDiff = Math.abs(this.getWristPosition() - position);
                return positionDiff < 23.0;
            });
    }

    public Command spinWheelsCommand(double velocity) {
        return this.run(() -> this.setShooterMotorVelocity(velocity))
            .until(() -> {
                double speedDiff = Math.abs(this.getShooterMotorVelocity() - velocity);
                return speedDiff < 1.0;
            });
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
        BaseStatusSignal.refreshAll(armPosition, wristAbsPosition, wristPosition, shooterVelocity, armFLVoltage, armFRVoltage,
            armBLVoltage, armBRVoltage, wristTopVoltage, wristBottomVoltage);   

        this.resetWristMotorPosition(this.getWristAbsPosition());

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
    }
}
