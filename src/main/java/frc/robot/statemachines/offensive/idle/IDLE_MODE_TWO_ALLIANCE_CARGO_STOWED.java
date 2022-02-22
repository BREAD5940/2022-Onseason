package frc.robot.statemachines.offensive.idle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.statemachines.Superstructure;
import frc.robot.statemachines.offensive.shoot.SETUP_SHOT_TWO_ALLIANCE_CARGO;

public class IDLE_MODE_TWO_ALLIANCE_CARGO_STOWED extends CommandBase {

    private final Superstructure superstructure;

    public IDLE_MODE_TWO_ALLIANCE_CARGO_STOWED(Superstructure superstructure) {
        setName("IDLE_MODE_TWO_RED_CARGO_STOWED");
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        superstructure.idle();

        if (RobotContainer.controller.getAButton()) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new SETUP_SHOT_TWO_ALLIANCE_CARGO(superstructure));
        }
    }
    
}
