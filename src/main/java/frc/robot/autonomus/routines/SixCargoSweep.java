package frc.robot.autonomus.routines;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomus.Trajectories;
import frc.robot.commons.BreadUtil;
import frc.robot.statemachines.Superstructure;
import frc.robot.subsystems.intake.IntakePneumatics;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerController;

import static frc.robot.Constants.Autonomus.*;

public class SixCargoSweep extends SequentialCommandGroup {

    private final Superstructure superstructure;
    private final IntakePneumatics intakePneumatics;
    private final Swerve swerve;

    private volatile int ballCount = 0;
    private volatile boolean inCircularSweep = true;
    private volatile boolean advancingToHumanPlayerStation = false;
    private volatile boolean intakingFromHumanPlayerStation = false;
    private volatile boolean returningFromHumanPlayerStation = false;
    private volatile boolean returnedFromHumanPlayerStation = false;
    private final Timer timer = new Timer();

    private SixCargoSweepState systemState = SixCargoSweepState.INTAKE_FROM_RIGHT_WHILE_SHOOTING;


    public SixCargoSweep(Superstructure superstructure, IntakePneumatics intakePneumatics, Swerve swerve) {
        this.superstructure = superstructure;
        this.intakePneumatics = intakePneumatics;
        this.swerve = swerve;
        addRequirements(superstructure, intakePneumatics, swerve);
        addCommands(
            new TrajectoryFollowerController(
                Trajectories.allianceSideSemiCircle, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET).plus(getTargetOffset()),
                () -> Rotation2d.fromDegrees(-40.63), 
                swerve
            ),
            new WaitCommand(0.5).andThen(() -> {
                inCircularSweep = false;
                advancingToHumanPlayerStation = true;
            }),
            new TrajectoryFollowerController(
                Trajectories.advanceToHumanPlayerStation, 
                (point, time) -> Rotation2d.fromDegrees(MathUtil.clamp(81.28 + (135 - 81.28) * time/Trajectories.advanceToHumanPlayerStation.getTotalTimeSeconds(), 81.28, 135)), 
                () -> swerve.getPose().getRotation(), 
                swerve
            ).andThen(() -> {
                advancingToHumanPlayerStation = false;
                intakingFromHumanPlayerStation = true;
            }),
            new WaitCommand(1.0).andThen(() -> {
                intakingFromHumanPlayerStation = false;
                returningFromHumanPlayerStation = true;
            }),
            new TrajectoryFollowerController(
                Trajectories.returnFromHumanPlayerStation, 
                (point, time) -> BreadUtil.getAngleToTarget(point.getTranslation(), FIELD_TO_TARGET), 
                () -> swerve.getPose().getRotation(), 
                swerve
            ).andThen(() -> {
                returningFromHumanPlayerStation = false;
                returnedFromHumanPlayerStation = true;
            }),
            new WaitCommand(2.0)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        super.execute();
         switch (systemState) {
            case INTAKE_FROM_RIGHT_WHILE_SHOOTING:
                intakePneumatics.extendRight();
                intakePneumatics.retractLeft();
                superstructure.dualIntake.setLeft(-1.0);
                superstructure.dualIntake.setRight(1.0);
                superstructure.gut.setSurfaceSpeed(-1.5);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(CIRCULAR_SWEEP_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(CIRCULAR_SWEEP_HOOD_ANGLE);
                if (superstructure.gut.getMiddleBeamBreak()&&superstructure.gut.getColorSensor()==RobotContainer.allianceColor) 
                    systemState = SixCargoSweepState.STOW_ALLIANCE_CARGO_IN_GUT_TO_SHOOT;
                if (advancingToHumanPlayerStation)
                    systemState = SixCargoSweepState.INTAKE_FROM_LEFT_AT_HUMAN_PLAYER_STATION_NO_CARGO;
                break;
            case STOW_ALLIANCE_CARGO_IN_GUT_TO_SHOOT:
                superstructure.dualIntake.setLeft(-1.0);
                superstructure.dualIntake.setRight(1.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(CIRCULAR_SWEEP_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(CIRCULAR_SWEEP_HOOD_ANGLE);
                if (superstructure.neck.getTopBeamBreak()) 
                    systemState = SixCargoSweepState.FIRST_WAIT_TO_SHOOT;
                break;
            case SHOOT_ALLIANCE_CARGO_TO_INTAKE: 
                superstructure.dualIntake.setLeft(-1.0);
                superstructure.dualIntake.setRight(1.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(CIRCULAR_SWEEP_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(CIRCULAR_SWEEP_HOOD_ANGLE);
                if (!superstructure.neck.getTopBeamBreak())  {
                    systemState = SixCargoSweepState.INTAKE_FROM_RIGHT_WHILE_SHOOTING;
                }
                break;
            case FIRST_WAIT_TO_SHOOT: 
                superstructure.dualIntake.setLeft(-1.0);
                superstructure.dualIntake.setRight(1.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(CIRCULAR_SWEEP_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(CIRCULAR_SWEEP_HOOD_ANGLE);
                if (timer.get() > 2.0) {
                    ballCount += 1;
                    systemState = SixCargoSweepState.SHOOT_ALLIANCE_CARGO_TO_INTAKE;
                }
                break;
            case INTAKE_FROM_LEFT_AT_HUMAN_PLAYER_STATION_NO_CARGO: 
                intakePneumatics.extendLeft();
                intakePneumatics.extendRight();
                superstructure.dualIntake.setLeft(1.0);
                superstructure.dualIntake.setRight(-1.0);
                superstructure.gut.setSurfaceSpeed(1.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(0.0);
                superstructure.hood.setPosition(0.0);
                if (superstructure.gut.getMiddleBeamBreak()) {
                    systemState = SixCargoSweepState.STOW_ALLIANCE_CARGO_WHILE_INTAKING_FROM_LEFT;
                }
                break;
            case STOW_ALLIANCE_CARGO_WHILE_INTAKING_FROM_LEFT:
                superstructure.dualIntake.setLeft(1.0);
                superstructure.dualIntake.setRight(-1.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(0.0);
                superstructure.hood.setPosition(0.0);
                if (superstructure.neck.getTopBeamBreak()) {
                    systemState = SixCargoSweepState.INTAKE_FROM_LEFT_AT_HUMAN_PLAYER_STATION_ONE_CARGO_STOWED;
                }
                break;
            case INTAKE_FROM_LEFT_AT_HUMAN_PLAYER_STATION_ONE_CARGO_STOWED:
                superstructure.dualIntake.setLeft(1.0);
                superstructure.dualIntake.setRight(-1.0);
                superstructure.gut.setSurfaceSpeed(1.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(RETURN_SHOT_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(RETURN_SHOT_HOOD_ANGLE);
                if (superstructure.gut.getMiddleBeamBreak())
                    systemState = SixCargoSweepState.SPIN_UP_AFTER_HPS;
                if (returnedFromHumanPlayerStation) {
                    systemState = SixCargoSweepState.SHOOT_AFTER_HUMAN_PLAYER_STATION;
                }
                break;
            case SPIN_UP_AFTER_HPS: 
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(0.0);
                superstructure.flywheel.setVelocity(RETURN_SHOT_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(RETURN_SHOT_HOOD_ANGLE);
                if (returnedFromHumanPlayerStation) {
                    systemState = SixCargoSweepState.SHOOT_AFTER_HUMAN_PLAYER_STATION;
                }
                break;
            case SHOOT_AFTER_HUMAN_PLAYER_STATION: 
                superstructure.dualIntake.setLeft(0.0);
                superstructure.dualIntake.setRight(0.0);
                superstructure.gut.setSurfaceSpeed(0.0);
                superstructure.neck.setSurfaceSpeed(3.0);
                superstructure.flywheel.setVelocity(RETURN_SHOT_FLYWHEEL_VELOCITY);
                superstructure.hood.setPosition(RETURN_SHOT_HOOD_ANGLE);
                break;


        }   
        SmartDashboard.putNumber("Ball Count", ballCount);
        System.out.println(systemState.name());
    }

    public enum SixCargoSweepState {
        STOW_ALLIANCE_CARGO_IN_GUT_TO_SHOOT,
        INTAKE_FROM_RIGHT_WHILE_SHOOTING,
        SHOOT_ALLIANCE_CARGO_TO_INTAKE,
        FIRST_WAIT_TO_SHOOT,
        INTAKE_FROM_LEFT_AT_HUMAN_PLAYER_STATION_NO_CARGO,
        INTAKE_FROM_LEFT_AT_HUMAN_PLAYER_STATION_ONE_CARGO_STOWED,
        STOW_ALLIANCE_CARGO_WHILE_INTAKING_FROM_LEFT,
        SPIN_UP_AFTER_HPS,
        SHOOT_AFTER_HUMAN_PLAYER_STATION
    }

    private Rotation2d getTargetOffset() {
        return new Rotation2d(Math.atan(swerve.getVelocity()*BALL_FLIGHT_TIME/RADIUS_TO_ROBOT_CENTER));
    }

    

}
