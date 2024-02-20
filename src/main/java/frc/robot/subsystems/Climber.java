package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    DoubleSolenoid climbLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2,6);
    DoubleSolenoid climbRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 7);



    public Climber() {

    }

    public void climbExtend() {
        climbLeft.set(Value.kForward);
        climbRight.set(Value.kForward);
    }

    public void climbRetract() {
        climbLeft.set(Value.kReverse);
        climbRight.set(Value.kReverse);
    }

}