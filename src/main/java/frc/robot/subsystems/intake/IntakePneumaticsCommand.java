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
    public void initialize() {
        intakePneumatics.retractLeft();
        intakePneumatics.retractRight();
    }

    @Override
    public void execute() {
        if (RobotContainer.operator.getLeftBumperPressed()) 
            intakePneumatics.extendLeft();
        if (RobotContainer.operator.getLeftBumperReleased())
            intakePneumatics.retractLeft();
        if (RobotContainer.operator.getRightBumperPressed())
            intakePneumatics.extendRight();
        if (RobotContainer.operator.getRightBumperReleased()) 
            intakePneumatics.retractRight();
    }


}