package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climber.Climber.ClimberStates;

public class ExtendToMidRung extends SequentialCommandGroup {

    public ExtendToMidRung(Climber climber) {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> climber.requestMidRungHeight(false, false, 1.4)),
            new WaitUntilCommand(() -> climber.getSystemState() == ClimberStates.NEUTRAL)
        );
    }
    
}
