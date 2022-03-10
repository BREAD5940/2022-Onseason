package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import static frc.robot.Constants.Hood.*;

public class Superstructure extends SubsystemBase {

    private final Shooter shooter;
    private final Intake leftIntake; 
    private final Intake rightIntake;
    private final GutNeck gutNeck;
    
    public Superstructure(Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        this.shooter = shooter;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.gutNeck = gutNeck;
    }

    @Override
    public void periodic() {
        if (RobotContainer.operator.getLeftStickButton()) {
            shooter.requestShoot(1350.0, HOOD_IDLE_POS);
        } else {
            shooter.requestIdle();
        }
        if (RobotContainer.operator.getAButton()) {
            shooter.requestShoot(1350.0, 23.0);
            gutNeck.requestShoot(true);
        } else {
            shooter.requestIdle();
            gutNeck.requestShoot(false);
        }
        if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
            leftIntake.requestIntake();
            rightIntake.requestOuttakeRetracted();
            gutNeck.requestIntakeLeft(true);
        } else if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
            leftIntake.requestOuttakeRetracted();
            rightIntake.requestIntake();
            gutNeck.requestIntakeRight(true);
        } else {
            gutNeck.requestIntakeLeft(false);
            gutNeck.requestIntakeLeft(false);
            if (RobotContainer.driver.getRawButtonPressed(9)) {
                leftIntake.requestIdleExtended();
            } else {
                leftIntake.requestIdleRetracted();
            }
            if (RobotContainer.driver.getRawButton(10)) {
                rightIntake.requestIdleExtended();
            } else {
                rightIntake.requestIdleRetracted();
            }
        } 

        if (RobotContainer.operator.getLeftBumper()) {
            gutNeck.requestSpitLeft(true);
            leftIntake.requestOuttakeRetracted();
            rightIntake.requestIdleRetracted();
        } else {
            gutNeck.requestSpitLeft(false);
        }

        if (RobotContainer.operator.getRightBumper()) {
            gutNeck.requestSpitRight(true);
            leftIntake.requestIdleRetracted();
            rightIntake.requestOuttakeRetracted();
        } else {
            gutNeck.requestSpitRight(false);
        }
    }

    

}
