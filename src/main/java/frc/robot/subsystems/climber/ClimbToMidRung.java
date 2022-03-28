package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climber.Climber.ClimberStates;

public class ClimbToMidRung extends SequentialCommandGroup {

    public ClimbToMidRung(Climber climber) {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> climber.requestRetracted(false, true, 1.5)),
            new WaitUntilCommand(() -> climber.getSystemState() == ClimberStates.NEUTRAL),
            new WaitCommand(0.4),
            new InstantCommand(() -> climber.requestHeightBeforeNextRung(false, false, 1.5)),
            new WaitUntilCommand(() -> climber.getSystemState() == ClimberStates.NEUTRAL)
        );
    }
    
}
