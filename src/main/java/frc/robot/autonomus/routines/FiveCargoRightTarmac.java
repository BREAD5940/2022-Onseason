package frc.robot.autonomus.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import frc.robot.subsystems.statemachines.GutNeck.GutNeckStates;
import frc.robot.subsystems.swerve.PointTurnCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;
import static frc.robot.Constants.Autonomus.*;

public class FiveCargoRightTarmac extends SequentialCommandGroup {  

    public FiveCargoRightTarmac(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        addRequirements(swerve, shooter, leftIntake, rightIntake, gutNeck);
        addCommands(
            new TrajectoryFollowerController(
                Trajectories.getFirstCargoRightTarmac, 
                (point, time) -> Rotation2d.fromDegrees(180.0), 
                () -> Rotation2d.fromDegrees(180.0), 
                swerve
            ).beforeStarting(
                () -> {
                    shooter.requestShoot(FIRST_SHOT_FLYWHEEL_VELOCITY, FIRST_SHOT_HOOD_ANGLE);
                    leftIntake.requestIntake();
                    gutNeck.requestIntakeLeft(true);
                }
            ),
            new WaitCommand(0.2),
            new TrajectoryFollowerController(
                Trajectories.returnFirstCargoRightTarmac, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET).plus(Rotation2d.fromDegrees(5.0)), 
                null,
                swerve
            ),
            new PointTurnCommand(
                () -> BreadUtil.getAngleToTarget(swerve.getPose().getTranslation(), FIELD_TO_TARGET).plus(Rotation2d.fromDegrees(5.0)).getRadians(), 
                swerve
            ),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
            }),
            new WaitUntilCommand(() -> gutNeck.getSystemState() == GutNeckStates.IDLE_NO_CARGO || gutNeck.getSystemState() == GutNeckStates.INTAKE_LEFT_NO_CARGO).andThen(
                () -> {
                    gutNeck.requestShoot(false);
                    shooter.requestShoot(SECOND_SHOT_FLYWHEEL_VELOCITY, SECOND_SHOT_HOOD_ANGLE);
                }
            ),
            new TrajectoryFollowerController(
                Trajectories.getThirdBallRightTarmac, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                null,
                swerve
            ),
            new PointTurnCommand(
                () -> BreadUtil.getAngleToTarget(swerve.getPose().getTranslation(), FIELD_TO_TARGET).getRadians(), 
                swerve
            ),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
            }),
            new WaitCommand(0.5).andThen(() -> gutNeck.requestShoot(true)), 
            new TrajectoryFollowerController(
                Trajectories.advanceToHumanPlayerStationAfterThreeBall,
                (point, time) -> Rotation2d.fromDegrees(135.0), 
                null, 
                swerve
            ).beforeStarting(() -> {
                gutNeck.requestShoot(false);
            }),
            new WaitCommand(0.25),
            new TrajectoryFollowerController(
                Trajectories.returnFromHumanPlayerStationAfterThreeBall,
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET).plus(Rotation2d.fromDegrees(5.0)), 
                null, 
                swerve
            ),
            new PointTurnCommand(
                () -> BreadUtil.getAngleToTarget(swerve.getPose().getTranslation(), FIELD_TO_TARGET).plus(Rotation2d.fromDegrees(5.0)).getRadians(), 
                swerve
            ),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
            })
        );
    }
 
}
