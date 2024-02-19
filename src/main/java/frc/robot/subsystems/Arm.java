package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    TalonFX m_armFL = new TalonFX(3); // Front Left ID 3
    TalonFX m_armFR = new TalonFX(9); // Front Right ID 9
    TalonFX m_armBL = new TalonFX(4); // Back Left ID 4
    TalonFX m_armBR = new TalonFX(2); // Back Right ID 2

    Follower follow = new Follower(9, false); // Front right as main motor
    Follower followOppose = new Follower(9, true); // Front right as main motor, to oppose
    
    final DutyCycleOut m_motorRequest = new DutyCycleOut(0.0);

    public Arm() {
        m_armBR.setControl(follow); // Back right follows Front right
        m_armFL.setControl(followOppose); // Front left follows and opposes Front Right
        m_armBL.setControl(followOppose); // Back left follows and opposes Front Right
        
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
