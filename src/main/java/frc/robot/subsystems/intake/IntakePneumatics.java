package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DualIntake.*;

public class IntakePneumatics extends SubsystemBase {

    private final DoubleSolenoid leftSolenoids = new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_INTAKE_PISTON_CHANNELS[0], LEFT_INTAKE_PISTON_CHANNELS[1]);
    private final DoubleSolenoid rightSolenoids = new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_INTAKE_PISTON_CHANNELS[0], RIGHT_INTAKE_PISTON_CHANNELS[1]);
    public boolean leftRetracted = true;
    public boolean rightRetracted = true;

    public void extendLeft() {
        leftRetracted = false;
    }

    public void extendRight() {
        rightRetracted = false;
    }

    public void retractLeft() {
        leftRetracted = true;
    }

    public void retractRight() {
        rightRetracted = true;
    }
    
    @Override
    public void periodic() {
        if (leftRetracted) {
            leftSolenoids.set(Value.kReverse);
        } else {
            leftSolenoids.set(Value.kForward);
        }

        if (rightRetracted) {
            rightSolenoids.set(Value.kReverse);
        } else {
            rightSolenoids.set(Value.kForward);
        }
    }
}
