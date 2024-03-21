package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // Solenoids for the climber
    // Left Climber solenoid (channel 6 forward, channel 2 reverse)
    private final DoubleSolenoid climbLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6,2);
    // Right Climber solenoid (channel 7 forward, channel 3 reverse)
    private final DoubleSolenoid climbRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 3);

    public Climber() { }

    // Extend the climber
    public void climbExtend() {
        // Set the solenoids forward to extend
        climbLeft.set(Value.kForward);
        climbRight.set(Value.kForward);
    }

    // Retract the climber
    public void climbRetract() {
        // Set the solenoids reverse to retract
        climbLeft.set(Value.kReverse);
        climbRight.set(Value.kReverse);
    }
}
