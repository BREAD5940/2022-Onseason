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

public class TwoCargoLeftTarmacDefensive extends SequentialCommandGroup {

    public TwoCargoLeftTarmacDefensive(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
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
                Trajectories.getFirstOpposingCargoTwoCargoDefensive, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                null,
                swerve
            ),
            new WaitCommand(0.8).andThen(() -> {
                leftIntake.requestIdleExtended();
                rightIntake.requestIntake();
                gutNeck.requestIntakeLeft(false);
                gutNeck.requestIntakeRight(true);
            }),
            new TrajectoryFollowerController(
                Trajectories.getSecondOpposingCargoTwoCargoDefensive, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                null,
                swerve
            ),
            new WaitCommand(0.2),
            new TrajectoryFollowerController(
                Trajectories.spitOpposingCargoTwoCargoDefensive, 
                (point, time) -> Rotation2d.fromDegrees(90.0), 
                null,
                swerve
            ),
            new InstantCommand(() ->  {
                leftIntake.requestOuttakeExtended(true);
                gutNeck.requestSpitLeft(true);
            }),
            new WaitCommand(2.0),
            new InstantCommand(() -> {
                leftIntake.requestIdleRetracted();
                rightIntake.requestIdleRetracted();
                gutNeck.requestIntakeLeft(false);
                gutNeck.requestIntakeRight(false);
                gutNeck.requestSpitLeft(false);
            })
        );
    }
    
}
