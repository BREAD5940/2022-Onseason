package frc.robot.autonomus.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.statemachines.Superstructure;
import frc.robot.subsystems.intake.IntakePneumatics;
import frc.robot.subsystems.swerve.PointTurnCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;
import static frc.robot.Constants.Autonomus.*;

public class TwoCargoTurnToRight extends SequentialCommandGroup {

    private final Superstructure superstructure;
    private final IntakePneumatics intakePneumatics;
    private final Swerve swerve;
    private boolean drivingToBall = true;
    private boolean setupShot = false;
    private boolean finishedShooting = false;
    private Timer shootingTimer = new Timer();
    private TwoCargoTurnToRightStates systemState = TwoCargoTurnToRightStates.INTAKE_FROM_LEFT_ONE_CARGO;

    public TwoCargoTurnToRight(Superstructure superstructure, IntakePneumatics intakePneumatics, Swerve swerve) {
        this.superstructure = superstructure;
        this.intakePneumatics = intakePneumatics;
        this.swerve = swerve;
        addRequirements(superstructure, intakePneumatics, swerve);
        addCommands(
            new TrajectoryFollowerController(
                Trajectories.twoCargoFromBackRight, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                () -> Rotation2d.fromDegrees(-43.57999897003174), 
                swerve
            ).andThen(() -> {
                drivingToBall = false;
                setupShot = true;
            }),
            new PointTurnCommand(
                () -> BreadUtil.getAngleToTarget(swerve.getPose().getTranslation(), FIELD_TO_TARGET).getRadians(), 
                swerve
            ),
            new WaitUntilCommand(() -> finishedShooting)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        shootingTimer.reset();
        intakePneumatics.extendRight();
        intakePneumatics.retractLeft();
        System.out.println(">>>>Command Started");
    }

    @Override
    public void execute() {
        super.execute();
        switch (systemState) {
            case INTAKE_FROM_LEFT_ONE_CARGO:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(1.0);
                superstructure.gut.setSurfaceSpeed(-1.5);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(TWO_SHOT_FLYWHEL_VELOCITY);
                superstructure.hood.setPosition(TWO_SHOT_HOOD_ANGLE);
                if (superstructure.gut.getMiddleBeamBreak()) {
                    intakePneumatics.retractLeft();
                    intakePneumatics.retractRight();
                    systemState = TwoCargoTurnToRightStates.DO_NOTHING;
                }
                break;
            case DO_NOTHING:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(TWO_SHOT_FLYWHEL_VELOCITY);
                superstructure.hood.setPosition(TWO_SHOT_HOOD_ANGLE);
                if (setupShot) 
                    systemState = TwoCargoTurnToRightStates.SETUP_SHOT;
                break;
            case SETUP_SHOT:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(TWO_SHOT_FLYWHEL_VELOCITY);
                superstructure.hood.setPosition(TWO_SHOT_HOOD_ANGLE);
                if (superstructure.flywheel.atSetpoint()&&superstructure.hood.atReference()) {
                    shootingTimer.start();
                    systemState = TwoCargoTurnToRightStates.SHOOT;
                }
                break;
            case SHOOT:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(1.0);
                superstructure.flywheel.setVelocity(TWO_SHOT_FLYWHEL_VELOCITY);
                superstructure.hood.setPosition(TWO_SHOT_HOOD_ANGLE);
                if (shootingTimer.get() > 1.5 && !superstructure.neck.getTopBeamBreak()) {
                    superstructure.dualIntake.setLeft(0.0);
                    superstructure.dualIntake.setRight(0.0);
                    superstructure.gut.setSurfaceSpeed(0.0);
                    superstructure.neck.setSurfaceSpeed(0.0);
                    superstructure.flywheel.setVelocity(0.0);
                    superstructure.hood.setPosition(0.0);
                    finishedShooting = true;
                }
                break;
        }
    }

    public enum TwoCargoTurnToRightStates {
        INTAKE_FROM_LEFT_ONE_CARGO, DO_NOTHING, SETUP_SHOT, SHOOT
    }


    
}
