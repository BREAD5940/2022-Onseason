package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climber.Climber.ClimberStates;

public class ReadyForHighRung extends SequentialCommandGroup {

    public ReadyForHighRung(Climber climber) {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> climber.requestReadyForNextRung(true, false, 1.5)),
            new WaitUntilCommand(() -> climber.getSystemState() == ClimberStates.NEUTRAL)
        );
    }
    
}