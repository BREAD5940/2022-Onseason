package frc.robot.statemachines.offensive.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.statemachines.Superstructure;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_TWO_ALLIANCE_CARGO_STOWED;

public class SETUP_SHOT_TWO_ALLIANCE_CARGO extends CommandBase {

    private final Superstructure superstructure;

    public SETUP_SHOT_TWO_ALLIANCE_CARGO(Superstructure superstructure) {
        setName("SETUP_SHOT_TWO_ALLIANCE_CARGO");
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        superstructure.spinup();

        if (superstructure.flywheel.atSetpoint()&&superstructure.hood.atReference()) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new SHOOT_TWO_ALLIANCE_CARGO(superstructure));
        } else if (!RobotContainer.operator.getAButton()) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new IDLE_MODE_TWO_ALLIANCE_CARGO_STOWED(superstructure));
        }

    }
    
}