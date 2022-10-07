package frc.robot.autonomus.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;
import static frc.robot.Constants.Autonomus.*;

public class OneCargoLeftTarmac extends SequentialCommandGroup {
   
    public OneCargoLeftTarmac(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        addRequirements(swerve, shooter, leftIntake, rightIntake, gutNeck);
        addCommands(
            new InstantCommand(()  -> { 
                gutNeck.requestShoot(true);
                shooter.requestShoot(BUMPER_FLYWHEEL_VELOCITY, BUMPER_HOOD_ANGLE);
            }),
            new WaitCommand(2),
            new TrajectoryFollowerController(
                Trajectories.goBackFromLeftTarmac,
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                () -> Rotation2d.fromDegrees(-45),
                 swerve
             )
        );

    }

}