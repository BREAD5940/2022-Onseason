package frc.robot.autonomus.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class TwoCargoLeftTarmacDefensive extends SequentialCommandGroup {

    public TwoCargoLeftTarmacDefensive(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        addRequirements(swerve, shooter, leftIntake, rightIntake, gutNeck);
        addCommands(  
        );
    }
    
}
