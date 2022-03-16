package frc.robot.autonomus.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;
import static frc.robot.Constants.Autonomus.*;

public class ThreeCargoRightTarmac extends SequentialCommandGroup {

    public ThreeCargoRightTarmac(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        addRequirements(swerve, shooter, leftIntake, rightIntake, gutNeck);
        addCommands(
            new TrajectoryFollowerController(
                Trajectories.getFirstCargoRightTarmac, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                () -> Rotation2d.fromDegrees(87.52), 
                swerve
            ).beforeStarting(
                () -> {
                    shooter.requestShoot(THREE_SHOT_FLYWHEEL_VELOCITY, THREE_SHOT_HOOD_ANGLE);
                    leftIntake.requestIntake();
                    gutNeck.requestIntakeLeft(true);
                }
            ),
            new InstantCommand(() -> {
                shooter.requestShoot(THREE_SHOT_FLYWHEEL_VELOCITY, THREE_SHOT_HOOD_ANGLE);
                gutNeck.requestShoot(true);
            })
        );
    }
    
}
