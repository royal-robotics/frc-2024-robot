package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    TalonFX m_armFL = new TalonFX(3); // Front Left ID 3
    TalonFX m_armFR = new TalonFX(9); // Front Right ID 9
    TalonFX m_armBL = new TalonFX(4); // Back Left ID 4
    TalonFX m_armBR = new TalonFX(2); // Back Right ID 2
    
    TalonFX m_shooter = new TalonFX(5); // Shooter motor ID 5
    TalonFX m_intake = new TalonFX(6); // Intake motor ID 6
    TalonFX m_wristBottom = new TalonFX(7); // wrist Bottom ID 7
    TalonFX m_wristTop = new TalonFX(8); // Wrist top ID 8

    Follower follow = new Follower(9, false); // Front right as main motor
    Follower followOppose = new Follower(9, true); // Front right as main motor, to oppose
    
    Follower followWrist = new Follower(8, false); // Wrist top is to be folowed

    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    MotorOutputConfigs motorConfigsReversed = new MotorOutputConfigs();

    final DutyCycleOut m_motorRequest = new DutyCycleOut(0.0);

    // in init function, set slot 0 gains
    Slot0Configs slot0Configs = new Slot0Configs();

    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Arm() {
        slot0Configs.kP = 2.4; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.0; // A velocity of 1 rps results in 0.1 V output

        m_armFR.getConfigurator().apply(slot0Configs);
        m_wristBottom.getConfigurator().apply(slot0Configs);
        m_wristTop.getConfigurator().apply(slot0Configs);

        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.NeutralMode = NeutralModeValue.Brake;
        motorConfigsReversed.Inverted = InvertedValue.Clockwise_Positive;

        m_armFL.getConfigurator().apply(motorConfigs);
        m_armBL.getConfigurator().apply(motorConfigs);
        m_armFR.getConfigurator().apply(motorConfigs);
        m_armBR.getConfigurator().apply(motorConfigs);

        m_shooter.getConfigurator().apply(motorConfigs);
        m_intake.getConfigurator().apply(motorConfigsReversed);
        m_wristBottom.getConfigurator().apply(motorConfigsReversed);
        m_wristTop.getConfigurator().apply(motorConfigsReversed);
        
        m_armBR.setControl(follow); // Back right follows Front right
        m_armFL.setControl(followOppose); // Front left follows and opposes Front Right
        m_armBL.setControl(followOppose); // Back left follows and opposes Front Right
        m_wristBottom.setControl(followWrist); // Bottom wrist follows top

        m_orchestra.addInstrument(m_armFL);
        System.out.println(m_armFL);
        m_orchestra.addInstrument(m_armFR);
        m_orchestra.addInstrument(m_armBL);
        m_orchestra.addInstrument(m_armBR);
        m_orchestra.addInstrument(m_shooter);
        m_orchestra.addInstrument(m_intake);
        m_orchestra.addInstrument(m_wristBottom);
        m_orchestra.addInstrument(m_wristTop);
        
        // Attempt to load the chrp
        StatusCode status = m_orchestra.loadMusic("deploy/Kevins Great File.chrp");

        if (!status.isOK()) {
        // log error
            System.out.println("error!");
        }

        m_orchestra.play();
    }

    public void motorSetArmPosition() {
        // set position to 10 rotations
        m_armFR.setControl(m_request.withPosition(10));
    }

    public void motorSetWristPosition() {
        // set position to 10 rotations
        m_wristTop.setControl(m_request.withPosition(10));
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

    @Override
    public void periodic() {

    }

}
