package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climber.Climber.ClimberStates;

public class PopOffStaticHooks extends SequentialCommandGroup {

    public PopOffStaticHooks(Climber climber) {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> climber.requestPullOffHeight(true, true, 1.5)),
            new WaitUntilCommand(() -> climber.getSystemState() == ClimberStates.NEUTRAL)
        );
    }
    
}
