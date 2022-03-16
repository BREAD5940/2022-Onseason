package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ClimbToHighRung extends SequentialCommandGroup {

    public ClimbToHighRung(Climber climber) {
        addRequirements(climber);
        addCommands(
            new PopOffStaticHooks(climber),
            new WaitCommand(0.250),
            new ClimbToNextRung(climber)
        );
    }
    
}