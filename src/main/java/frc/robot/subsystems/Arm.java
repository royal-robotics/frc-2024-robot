package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    TalonFX m_armFL = new TalonFX(3); // Front Left ID 3
    TalonFX m_armFR = new TalonFX(9); // Front Right ID 9
    TalonFX m_armBL = new TalonFX(4); // Back Left ID 4
    TalonFX m_armBR = new TalonFX(2); // Back Right ID 2
    
    CANcoder armEncoder = new CANcoder(5); 
    double encoderOffset = 16.0;

    TalonFX m_shooter = new TalonFX(5); // Shooter motor ID 5
    TalonFX m_intake = new TalonFX(6); // Intake motor ID 6
    TalonFX m_wristBottom = new TalonFX(7); // wrist Bottom ID 7
    TalonFX m_wristTop = new TalonFX(8); // Wrist top ID 8

    Follower follow = new Follower(9, false); // Front right as main motor
    Follower followOppose = new Follower(9, true); // Front right as main motor, to oppose
    
    Follower followWrist = new Follower(8, false); // Wrist top is to be folowed

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    MotorOutputConfigs motorConfigsReversed = new MotorOutputConfigs();

    MagnetSensorConfigs encoderConfigs = new MagnetSensorConfigs();

    final DutyCycleOut m_motorRequest = new DutyCycleOut(0.0);

    // in init function, set slot 0 gains
    Slot0Configs slot0Configs = new Slot0Configs();

    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public static Orchestra m_orchestra = new Orchestra("Kevins Great File.chrp");
    MusicTone musicFreq = new MusicTone(256); // 256 hz
    AudioConfigs audioConfigs = new AudioConfigs();

    public AnalogInput lineBreakSensor = new AnalogInput(3); // Linebreak Sensor on channel 3
    public DigitalInput armLimitZero = new DigitalInput(1); // Arm limit switch for arm at 0 on channel 1

    public StatusSignal<Double> armPosition;
    public StatusSignal<Double> wristPosition;
    public StatusSignal<Double> armAbsPosition;


    public Arm() {
        slot0Configs.kP = 0.24; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

        m_armFR.getConfigurator().apply(slot0Configs);
        m_wristBottom.getConfigurator().apply(slot0Configs);
        m_wristTop.getConfigurator().apply(slot0Configs);

        encoderConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;

        m_armFL.getConfigurator().apply(motorConfigs);
        m_armBL.getConfigurator().apply(motorConfigs);
        m_armFR.getConfigurator().apply(motorConfigs);
        m_armBR.getConfigurator().apply(motorConfigs);

        armEncoder.getConfigurator().apply (encoderConfigs);

        m_shooter.getConfigurator().apply(motorConfigs);
        m_intake.getConfigurator().apply(motorConfigsReversed);
        m_wristBottom.getConfigurator().apply(motorConfigsReversed);
        m_wristTop.getConfigurator().apply(motorConfigsReversed);
        
        m_armBR.setControl(follow); // Back right follows Front right
        m_armFL.setControl(followOppose); // Front left follows and opposes Front Right
        m_armBL.setControl(followOppose); // Back left follows and opposes Front Right
        m_wristBottom.setControl(followWrist); // Bottom wrist follows top

        audioConfigs.AllowMusicDurDisable = false;

        m_orchestra.addInstrument(m_armFL);
        System.out.println("The arm motor is " + m_armFL);
        // m_orchestra.addInstrument(m_armFR);
        // m_orchestra.addInstrument(m_armBL);
        // m_orchestra.addInstrument(m_armBR);
        // m_orchestra.addInstrument(m_shooter);
        // m_orchestra.addInstrument(m_intake);
        // m_orchestra.addInstrument(m_wristBottom);
        // m_orchestra.addInstrument(m_wristTop);
        
        // Attempt to load the chrp
        StatusCode status = m_orchestra.loadMusic("Kevins Great File.chrp"); // Moved to object declaration

        if (!status.isOK()) {
        // log error
           System.out.println("error in arm!");
        }

        // m_orchestra.play();

        armPosition = m_armFR.getPosition();
        armAbsPosition = armEncoder.getAbsolutePosition();
        wristPosition = m_wristTop.getPosition();
    }

    public void motorSetArmPosition() {
        // set position to 10 rotations
        m_armFR.setControl(m_request.withPosition(0));
    }

    public void motorSetWristPosition() {
        // set position to 10 rotations
        m_wristTop.setControl(m_request.withPosition(0));
    }

    public void motorRequestForward() {
        m_motorRequest.Output = 0.1;
        m_armFR.setControl(m_motorRequest);
    }
    
    public void motorRequestReverse() {
        m_motorRequest.Output = -0.1;
        m_armFR.setControl(m_motorRequest);
    }

    public void motorRequestBrake() {
        m_motorRequest.Output = 0.0;
        m_armFR.setControl(m_motorRequest);
    }

    public void requestIntake() {
        m_motorRequest.Output = 0.6;
        m_intake.setControl(m_motorRequest);
    }

    public void requestShooter() {
        m_motorRequest.Output = 0.9;
        m_shooter.setControl(m_motorRequest);
    }

    public void requestInShootStop() {
        m_motorRequest.Output = 0.0;
        m_shooter.setControl(m_motorRequest);
        m_intake.setControl(m_motorRequest);
    }

    public boolean brokenLine() {
        if (lineBreakSensor.getVoltage() > 3) {
            return true;
        } else {
            return false;
        }
    }

    public void setMotorBrake(){ 
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Brake;
        m_armFL.getConfigurator().apply(motorConfigs);
        m_armBL.getConfigurator().apply(motorConfigs);
        m_armFR.getConfigurator().apply(motorConfigs);
        m_armBR.getConfigurator().apply(motorConfigs);

        m_shooter.getConfigurator().apply(motorConfigs);
        m_intake.getConfigurator().apply(motorConfigsReversed);
        m_wristBottom.getConfigurator().apply(motorConfigsReversed);
        m_wristTop.getConfigurator().apply(motorConfigsReversed);
    }

    public void setMotorCoast(){
        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Coast;
        m_armFL.getConfigurator().apply(motorConfigs);
        m_armBL.getConfigurator().apply(motorConfigs);
        m_armFR.getConfigurator().apply(motorConfigs);
        m_armBR.getConfigurator().apply(motorConfigs);

        m_shooter.getConfigurator().apply(motorConfigs);
        m_intake.getConfigurator().apply(motorConfigsReversed);
        m_wristBottom.getConfigurator().apply(motorConfigsReversed);
        m_wristTop.getConfigurator().apply(motorConfigsReversed);
    }

    public double wristRotations() {
        return armAbsPosition.getValue() + Units.degreesToRotations(encoderOffset);
    }

    public void resetWristMotor() {
        double motorPosition = wristRotations() * (84 / 12) * (84 / 18);
        m_wristTop.setPosition(motorPosition);
    }

    @Override
    public void periodic() {
        armPosition.refresh();
        armAbsPosition.refresh();
        wristPosition.refresh();
        
    }

}
