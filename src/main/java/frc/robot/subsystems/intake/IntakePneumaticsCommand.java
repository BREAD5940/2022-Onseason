package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakePneumaticsCommand extends CommandBase {

    private final IntakePneumatics intakePneumatics;

    public IntakePneumaticsCommand(IntakePneumatics intakePneumatics) {
        this.intakePneumatics = intakePneumatics;
        addRequirements(intakePneumatics);
    }

    @Override
    public void execute() {
        if (RobotContainer.controller.getLeftBumperPressed()) {
            if (intakePneumatics.leftRetracted) {
                intakePneumatics.extendLeft();
            } else {
                intakePneumatics.retractLeft();
            }
        }

        if (RobotContainer.controller.getRightBumperPressed()) {
            if (intakePneumatics.rightRetracted) {
                intakePneumatics.extendRight();
            } else {
                intakePneumatics.retractRight();
            }
        }
    }


}