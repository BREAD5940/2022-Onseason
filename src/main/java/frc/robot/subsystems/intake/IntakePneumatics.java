package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DualIntake.*;

public class IntakePneumatics extends SubsystemBase {

    private final DoubleSolenoid leftSolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, LEFT_INTAKE_PISTON_CHANNELS[0], LEFT_INTAKE_PISTON_CHANNELS[1]);
    private final DoubleSolenoid rightSolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RIGHT_INTAKE_PISTON_CHANNELS[0], RIGHT_INTAKE_PISTON_CHANNELS[1]);

    public void extendLeft() {
        leftSolenoids.set(Value.kForward);
    }

    public void extendRight() {
        System.out.println("Extend right requested");
        rightSolenoids.set(Value.kForward);
    }

    public void retractLeft() {
        leftSolenoids.set(Value.kReverse);
    }

    public void retractRight() {
        System.out.println("Retract right requested");
        rightSolenoids.set(Value.kReverse);
    }
    
}
