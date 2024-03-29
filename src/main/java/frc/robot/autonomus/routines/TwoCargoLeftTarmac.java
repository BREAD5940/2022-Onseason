package frc.robot.autonomus.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;
import frc.robot.subsystems.swerve.VisionFollowerController;

import static frc.robot.Constants.Autonomus.*;

public class TwoCargoLeftTarmac extends SequentialCommandGroup {

    public TwoCargoLeftTarmac(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        addRequirements(swerve, shooter, leftIntake, rightIntake, gutNeck);
        addCommands(  
            new TrajectoryFollowerController(
                Trajectories.twoCargoLeftTarmac, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                () -> Rotation2d.fromDegrees(1.45), 
                swerve
            ).beforeStarting(
                () -> {
                    shooter.requestShoot(TWO_SHOT_FLYWHEL_VELOCITY, TWO_SHOT_HOOD_ANGLE);
                    leftIntake.requestIntake();
                    gutNeck.requestIntakeLeft(true);
                }
            ),
            new WaitUntilCommand(() -> swerve.getAtVisionHeadingSetpoint()).alongWith(
                new VisionFollowerController(swerve, true)
            ).withTimeout(0.5).beforeStarting(() -> RobotContainer.swerve.setDriveSlots(0))
            .andThen(() -> RobotContainer.swerve.setDriveSlots(1)),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
            })
        );
    }
    
}
