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

public class BilliardsThreeBallLeftTarmac extends SequentialCommandGroup {

    public BilliardsThreeBallLeftTarmac(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
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
            new PointTurnCommand(
                () -> BreadUtil.getAngleToTarget(swerve.getPose().getTranslation(), FIELD_TO_TARGET).getRadians(), 
                swerve
            ),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
            }), 
            new WaitCommand(1.0),
            new WaitUntilCommand(() -> gutNeck.getSystemState() == GutNeckStates.IDLE_NO_CARGO || gutNeck.getSystemState() == GutNeckStates.INTAKE_LEFT_NO_CARGO).andThen(() -> {
                gutNeck.requestShoot(false);
                shooter.requestIdle();
                gutNeck.acceptOpposingCargo(true);
            }),
            new TrajectoryFollowerController(
                Trajectories.getFirstOpposingCargoBilliards, 
                (point, time) -> Rotation2d.fromDegrees(22.5), 
                null, 
                swerve
            ),
            new WaitCommand(1.5),
            new InstantCommand(() ->  {
                leftIntake.requestOuttakeExtended(true);
                gutNeck.requestSpitLeft(true);
            }),
            new WaitCommand(2.0).andThen(() -> {
                gutNeck.requestIntakeLeft(true);
                leftIntake.requestIntake();
            }),
            new TrajectoryFollowerController(
                Trajectories.getUnstagedBallBilliards, 
                (point, time) -> Rotation2d.fromDegrees(0.0), 
                null, 
                swerve
            ),
            new WaitCommand(1.0),
            new TrajectoryFollowerController(
                Trajectories.returnUnstagedBallBilliards, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                null, 
                swerve
            ).beforeStarting(
                () -> {
                    shooter.requestShoot(BILLIARDS_FLYWHEEL_VELOCITY, BILLIARDS_HOOD_ANGLE);
                    leftIntake.requestIntake();
                    gutNeck.requestIntakeLeft(true);
                }
            ),
            new PointTurnCommand(
                () -> BreadUtil.getAngleToTarget(swerve.getPose().getTranslation(), FIELD_TO_TARGET).getRadians(), 
                swerve
            ),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
            }), 
            new WaitCommand(1.0),
            new WaitUntilCommand(() -> gutNeck.getSystemState() == GutNeckStates.IDLE_NO_CARGO || gutNeck.getSystemState() == GutNeckStates.INTAKE_LEFT_NO_CARGO).andThen(() -> {
                gutNeck.requestShoot(false);
                shooter.requestIdle();
                gutNeck.acceptOpposingCargo(false);
                leftIntake.requestIdleRetracted();
            })
        );
    }
    
}
