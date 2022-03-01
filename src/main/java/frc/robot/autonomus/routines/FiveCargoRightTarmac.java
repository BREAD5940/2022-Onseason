package frc.robot.autonomus.routines;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.statemachines.Superstructure;
import frc.robot.subsystems.intake.IntakePneumatics;
import frc.robot.subsystems.swerve.Swerve;
import static frc.robot.Constants.Autonomus.*;

public class FiveCargoRightTarmac extends SequentialCommandGroup {

    private final Superstructure superstructure;
    private final IntakePneumatics intakePneumatics;
    private final Swerve swerve;
    private final Timer timeoutTimer = new Timer();
    private FiveCargoRightTarmacState systemState = FiveCargoRightTarmacState.SETUP_FIRST_CARGO_SHOT;

    private boolean intakedFirstBall = false;

    public FiveCargoRightTarmac(Superstructure superstructure, IntakePneumatics intakePneumatics, Swerve swerve) {
        this.superstructure = superstructure;
        this.intakePneumatics = intakePneumatics;
        this.swerve = swerve;
        addRequirements(superstructure, intakePneumatics, swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        intakePneumatics.retractRight();
        intakePneumatics.retractLeft();
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    @Override
    public void execute() {
        super.execute();

        switch (systemState) {
            case SETUP_FIRST_CARGO_SHOT: 
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(FIRST_CARGO_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(FIRST_CARGO_HOOD_ANGLE);
                if (superstructure.flywheel.atSetpoint()&&superstructure.hood.atReference()) {
                    systemState = FiveCargoRightTarmacState.SHOOT_FIRST_CARGO;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                }
                break;
            case SHOOT_FIRST_CARGO:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(FIRST_CARGO_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(FIRST_CARGO_HOOD_ANGLE);
                if (!superstructure.neck.getTopBeamBreak()&&!superstructure.gut.getMiddleBeamBreak()&&timeoutTimer.get()>1.0) {
                    systemState = FiveCargoRightTarmacState.SHOOT_FIRST_CARGO;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                }
                break;
            case INTAKE_FIRST_BALL_FROM_RIGHT: 
                superstructure.dualIntake.setLeft(1.0);
                superstructure.dualIntake.setRight(-1.0);
                superstructure.gut.setSurfaceSpeed(1.5);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(0.0);
                superstructure.hood.setPosition(0.0);
                if (superstructure.gut.getMiddleBeamBreak()) {
                    systemState = FiveCargoRightTarmacState.STOW_CARGO_AFTER_INTAKING_FIRST_BALL;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                    intakePneumatics.extendRight();
                    intakePneumatics.retractLeft();
                }
                break;
            case STOW_CARGO_AFTER_INTAKING_FIRST_BALL:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(0.0);
                superstructure.hood.setPosition(0.0);
                if (superstructure.neck.getTopBeamBreak()) {
                    systemState = FiveCargoRightTarmacState.INTAKE_SECOND_BALL_FROM_LEFT;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                    intakePneumatics.extendLeft();
                    intakePneumatics.retractRight();
                }
                break;
            case INTAKE_SECOND_BALL_FROM_LEFT:
                superstructure.dualIntake.setLeft(-1.0);
                superstructure.dualIntake.setRight(1.0);
                superstructure.gut.setSurfaceSpeed(-1.5);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(0.0);
                superstructure.hood.setPosition(0.0);
                if (superstructure.gut.getMiddleBeamBreak()) {
                    systemState = FiveCargoRightTarmacState.STOW_CARGO_AFTER_INTAKING_FIRST_BALL;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                    intakePneumatics.extendRight();
                    intakePneumatics.retractLeft();
                }
                break;
            case SETUP_SECOND_CARGO_SHOT: 
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(SECOND_CARGO_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(SECOND_CARGO_HOOD_ANGLE);
                if (!superstructure.neck.getTopBeamBreak()&&!superstructure.gut.getMiddleBeamBreak()&&timeoutTimer.get()>1.0) {
                    systemState = FiveCargoRightTarmacState.SHOOT_SECOND_CARGO;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                    intakePneumatics.extendRight();
                    intakePneumatics.retractLeft();
                }
                break;
            case SHOOT_SECOND_CARGO:
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(SECOND_CARGO_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(SECOND_CARGO_HOOD_ANGLE);
                if (!superstructure.neck.getTopBeamBreak()&&!superstructure.gut.getMiddleBeamBreak()&&timeoutTimer.get()>1.0) {
                    systemState = null;
                    timeoutTimer.reset();
                    timeoutTimer.start();
                    intakePneumatics.extendRight();
                    intakePneumatics.retractLeft();
                }
                break;
                
        }

    }

    public enum FiveCargoRightTarmacState {
        STOW_CARGO_AFTER_INTAKING_FIRST_BALL, 
        SETUP_FIRST_CARGO_SHOT, 
        SETUP_SECOND_CARGO_SHOT,
        SHOOT_FIRST_CARGO, 
        SHOOT_SECOND_CARGO,
        INTAKE_FIRST_BALL_FROM_RIGHT, 
        INTAKE_SECOND_BALL_FROM_LEFT
    }




    
}
