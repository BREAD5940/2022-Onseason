package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DualIntake.*;

public class DualIntake extends SubsystemBase {

    SingleIntake leftIntake = new SingleIntake(
        LEFT_INTAKE_ID, 
        true
    );
    SingleIntake rightIntake = new SingleIntake(
        RIGHT_INTAKE_ID,
        false
    );

    public void setLeft(double percent) {
        leftIntake.set(percent);
    }

    public void setRight(double percent) {
        rightIntake.set(percent);
    }
    
}
