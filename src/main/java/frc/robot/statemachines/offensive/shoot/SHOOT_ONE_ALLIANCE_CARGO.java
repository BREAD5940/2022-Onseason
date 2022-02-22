package frc.robot.statemachines.offensive.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.statemachines.Superstructure;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_NO_CARGO_STOWED;

public class SHOOT_ONE_ALLIANCE_CARGO extends CommandBase {

    private final Superstructure superstructure;
    private final Timer timer = new Timer();

    public SHOOT_ONE_ALLIANCE_CARGO(Superstructure superstructure) {
        setName("SHOOT_ONE_ALLIANCE_CARGO");
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        superstructure.shoot();

        if (!superstructure.gut.getMiddleBeamBreak()&&!superstructure.neck.getTopBeamBreak()&&timer.get()>2) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new IDLE_MODE_NO_CARGO_STOWED(superstructure));
        }

    }
    
}
