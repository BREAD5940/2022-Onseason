package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.climber.Climber.ClimberStates;


public class ClimbToHighRung extends SequentialCommandGroup {

    public ClimbToHighRung(Climber climber) {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> climber.requestRetracted(true, true, 1.5)),
            new WaitCommand(0.1),
            new InstantCommand(() -> climber.requestRetracted(false, true, 1.5)),
            new WaitUntilCommand(() -> climber.getSystemState() == ClimberStates.NEUTRAL)
        );
    }
    
}