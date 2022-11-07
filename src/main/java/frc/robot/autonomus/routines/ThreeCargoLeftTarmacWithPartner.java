package frc.robot.autonomus.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.gutneck.GutNeck;
import frc.robot.subsystems.gutneck.GutNeck.GutNeckStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.PointTurnCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;

import static frc.robot.Constants.Autonomus.*;

public class ThreeCargoLeftTarmacWithPartner extends SequentialCommandGroup {

    public ThreeCargoLeftTarmacWithPartner(Swerve swerve, Shooter shooter, Intake leftIntake, Intake rightIntake, GutNeck gutNeck) {
        addRequirements(swerve, shooter, leftIntake, rightIntake, gutNeck);
        addCommands(
            new InstantCommand(() -> {
                rightIntake.requestIntake();
                gutNeck.requestIntakeRight(true);
                gutNeck.requestIntakeLeft(false);
            }),
            new WaitCommand(2.0),
            new InstantCommand(() -> {
                gutNeck.requestShoot(true);
                shooter.requestShoot(BUMPER_FLYWHEEL_VELOCITY, BUMPER_HOOD_ANGLE);
            }),
            new WaitCommand(1.0).andThen(() -> {
                rightIntake.requestIdleRetracted();
                gutNeck.requestIntakeRight(false);
            }),
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
            })
        );
    }
    
}
