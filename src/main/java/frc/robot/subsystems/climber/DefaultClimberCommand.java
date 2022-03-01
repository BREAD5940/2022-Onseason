package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import static frc.robot.Constants.Climber.*;

public class DefaultClimberCommand extends CommandBase {

    Climber climber;
    double setpoint = 0.005;

    public DefaultClimberCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }   

    @Override
    public void execute() {

        // Control up/down movement of the climber
        if (Math.abs(RobotContainer.operator.getRightY()) > 0.2) {
            climber.setPercent(RobotContainer.operator.getRightY() * 0.5);
        } else {
            climber.setPercent(0.0);
        }

        // Control extension and retraction of the climber
        if (RobotContainer.operator.getXButtonPressed()) {
            climber.extend();
        } 
        if (RobotContainer.operator.getBButtonPressed()) {
            climber.retract();
        }

    }
    
}
