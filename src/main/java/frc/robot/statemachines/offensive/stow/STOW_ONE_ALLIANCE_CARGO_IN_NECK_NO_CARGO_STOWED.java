package frc.robot.statemachines.offensive.stow;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.statemachines.Superstructure;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED;

public class STOW_ONE_ALLIANCE_CARGO_IN_NECK_NO_CARGO_STOWED extends CommandBase {

    private final Superstructure superstructure;

    public STOW_ONE_ALLIANCE_CARGO_IN_NECK_NO_CARGO_STOWED(Superstructure superstructure) {
        setName("STOW_ONE_ALLIANCE_CARGO_IN_NECK");
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void execute() {
        superstructure.stowCargoInNeck();
        
        if (superstructure.neck.getTopBeamBreak()) {
            CommandScheduler.getInstance().cancel(this);
            CommandScheduler.getInstance().schedule(new IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED(superstructure));
        }
    }
    
}
