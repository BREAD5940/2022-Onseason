package frc.robot.statemachines.offensive.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.statemachines.Superstructure;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_TWO_ALLIANCE_CARGO_STOWED;

public class INTAKE_FROM_LEFT_ONE_CARGO_STOWED extends CommandBase {

    private final Superstructure superstructure;

    public INTAKE_FROM_LEFT_ONE_CARGO_STOWED(Superstructure superstructure) {
        setName("INTAKE_FROM_LEFT_ONE_CARGO_STOWED");
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        superstructure.intakeLeft();

        if (RobotContainer.operator.getLeftTriggerAxis() < 0.1) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED(superstructure));
        } else if (superstructure.gut.getMiddleBeamBreak()&&superstructure.gut.getColorSensor()==Robot.allianceColor) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new IDLE_MODE_TWO_ALLIANCE_CARGO_STOWED(superstructure));
        }
    }
    
}
