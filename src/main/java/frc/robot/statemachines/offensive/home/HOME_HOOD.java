package frc.robot.statemachines.offensive.home;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commons.BreadUtil;
import frc.robot.statemachines.Superstructure;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_NO_CARGO_STOWED;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED;

public class HOME_HOOD extends CommandBase {

    private final Superstructure superstructure;
    private final Timer timer = new Timer();

    public HOME_HOOD(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.hood.setCurrentLimits(5, 10);
        superstructure.hood.setPercent(-0.1);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (BreadUtil.atReference(superstructure.hood.getVelocity(), 0.0, 10.0, true) && timer.get() >= 0.25) {
            timer.stop();
            superstructure.hood.setCurrentLimits(40, 50.0);
            superstructure.hood.setPercent(0.0);
            superstructure.hood.reset();
            CommandScheduler.getInstance().cancel(this);
            if (superstructure.neck.getTopBeamBreak()) {
                CommandScheduler.getInstance().schedule(new IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED(superstructure));
            } else {
                CommandScheduler.getInstance().schedule(new IDLE_MODE_NO_CARGO_STOWED(superstructure));
            }
        }
    }
    
}
