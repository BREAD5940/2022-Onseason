package frc.robot.subsystems.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.sensors.BeamBreak;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensor.BallColor;
import frc.robot.subsystems.statemachines.Shooter.ShooterState;
import frc.robot.Robot;
import static frc.robot.Constants.Gut.*;
import static frc.robot.Constants.Neck.*;

public class GutNeck extends SubsystemBase {

    // Instantiate the gut hardware
    private final TalonFX gutMotor = TalonFXFactory.createDefaultTalon(GUT_ID);
    private final TalonFX neckMotor = TalonFXFactory.createDefaultTalon(NECK_ID);
    private final BeamBreak leftBeamBreak = new BeamBreak(LEFT_BEAM_BREAK_CHANNEL);
    private final BeamBreak rightBeamBreak = new BeamBreak(RIGHT_BEAM_BREAK_CHANNEL);
    private final BeamBreak middleBeamBreak = new BeamBreak(MIDDLE_BEAM_BREAK_CHANNEL);
    private final BeamBreak topBeamBreak = new BeamBreak(TOP_BEAM_BREAK_CHANNEL);
    private final ColorSensor colorSensor = new ColorSensor(
        COLOR_SENSOR_PORT,
        COLOR_SENSOR_RED_TARGET,
        COLOR_SENSOR_BLUE_TARGET,
        COLOR_SENSOR_NO_TARGET
    );

    // State logic
    private GutNeckStates systemState = GutNeckStates.IDLE_NO_CARGO;
    private Timer shootingTimer = new Timer();
    private boolean ballsExpelledFromNeck = false;

    // Statemachine inputs
    private boolean acceptOpposingCargo = false;
    private boolean requestIntakeLeft = false;
    private boolean requestIntakeRight = false;
    private boolean requestShoot = false;
    private boolean requestSpitRight = false;
    private boolean requestSpitLeft = false;

    public GutNeck() {
        // Configure the gut motor
        TalonFXConfiguration gutConfig = new TalonFXConfiguration();
        gutConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        gutConfig.slot0.kP = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0.1) * 1023.0; 
        gutConfig.slot0.kI = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0) * 1023.0;
        gutConfig.slot0.kD = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0) * 1023.0;
        gutConfig.slot0.kF = 1023.0/gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(MAX_GUT_SURFACE_SPEED);
        gutConfig.velocityMeasurementWindow = 8;
        gutMotor.setNeutralMode(NeutralMode.Brake);
        gutMotor.set(ControlMode.Velocity, 0.0);
        gutMotor.setInverted(TalonFXInvertType.Clockwise);
        gutMotor.configAllSettings(gutConfig);
        gutMotor.selectProfileSlot(0, 0);
        gutMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        gutMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        // Configure the neck motor
        TalonFXConfiguration neckConfig = new TalonFXConfiguration();
        neckConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        neckConfig.slot0.kP = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0.02) * 1023.0; // TODO check this
        neckConfig.slot0.kI = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0) * 1023.0;
        neckConfig.slot0.kD = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0) * 1023.0;
        neckConfig.slot0.kF = 1023.0/neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(MAX_NECK_SURFACE_SPEED);
        neckMotor.setNeutralMode(NeutralMode.Brake);
        neckMotor.setInverted(TalonFXInvertType.Clockwise);
        neckMotor.set(ControlMode.Velocity, 0.0);
        neckMotor.configAllSettings(neckConfig);
        neckMotor.selectProfileSlot(0, 0);
        neckMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        neckMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    }

    // Returns the output of the left beam break
    public boolean getLeftBeamBreakTriggered() {
        return leftBeamBreak.get();
    }

    // Returns the output of the right beam break 
    public boolean getRightBeamBreakTriggered() {
        return rightBeamBreak.get();
    }

    // Returns the output of the middle beam break
    public boolean getMiddleBeamBreakTriggered() {
        return middleBeamBreak.get();
    }

    // Returns the output of the top beam break
    public boolean getTopBeamBreakTriggered() {
        return topBeamBreak.get();
    }

    // Returns the output of the color sensors
    public BallColor getMiddleColor() {
        return colorSensor.get();
    }

    // Returns the surface speed of the gut
    public double getGutSurfaceSpeed() {
        return integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(gutMotor.getSelectedSensorVelocity());
    }

    // Returns the surface speed of the neck
    public double getNeckSurfaceSpeed() {
        return integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(neckMotor.getSelectedSensorVelocity());
    }

    // Sets the surface speed of the gut
    private void commandGutSurfaceSpeed(double speed) {
        gutMotor.set(ControlMode.Velocity, gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speed));
    }

    // Sets the surface speed of the neck
    private void commandNeckSurfaceSpeed(double speed) {
        neckMotor.set(ControlMode.Velocity, neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speed));
    }

    // Converts integrated sensor units to the neck's surface speed
    private double integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * (NECK_GEARING * (600.0/2048.0) * Math.PI * NECK_PULLEY_DIAMETER) / 60.0;
    }
    
    // Converts the gut's surface speed to integrated sensor units
    private double neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(double surfaceSpeed) {
        return surfaceSpeed * 60.0 / (NECK_GEARING * (600.0/2048.0) * Math.PI * NECK_PULLEY_DIAMETER);
    }

    // Converts integrated sensor units to the gut's surface speed
    private double integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((GUT_GEARING * (600.0/2048.0) * Math.PI * GUT_PULLEY_DIAMETER) / 60.0);
    }

    // Converts the gut's surface speed to integrated sensor units
    private double gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(double gutSurfaceSpeed) {
        return gutSurfaceSpeed * (60.0/(GUT_GEARING * (600.0/2048.0) * Math.PI * GUT_PULLEY_DIAMETER));
    }

    // Sets the gut neck statemachine to accept opposing alliance cargo
    public void acceptOpposingCargo(boolean set) {
        acceptOpposingCargo = set;
    }

    // Requests the gut to intake from the left 
    public void requestIntakeLeft(boolean set) {
        requestIntakeLeft = set;
    }

    // Requests the gut to intake from the right
    public void requestIntakeRight(boolean set) {
        requestIntakeRight = set;
    }
    
    // Requests the gut neck subsystem to shoot
    public void requestShoot(boolean set) {
        requestShoot = set;
    }

    // Requests the gut neck subsystem to spit out all the balls in its system to the left
    public void requestSpitLeft(boolean set) {
        requestSpitLeft = set;
    }

    // Requests the gut neck subsystem to spit out all the balls in its system to the right
    public void requestSpitRight(boolean set) {
        requestSpitRight = set;
    }

    // Returns the system state of the gut neck
    public GutNeckStates getSystemState() {
        return systemState;
    }

    // Gut Neck States
    public enum GutNeckStates {
        IDLE_NO_CARGO,
        IDLE_ONE_CARGO,
        IDLE_TWO_CARGO,
        INTAKE_LEFT_NO_CARGO,
        INTAKE_RIGHT_NO_CARGO,
        INTAKE_LEFT_ONE_CARGO,
        INTAKE_RIGHT_ONE_CARGO, 
        STOW_ONE_CARGO_IN_NECK,
        SHOOT_CARGO,
        SPIT_LEFT, 
        SPIT_RIGHT
    } 

    // Update the state machine
    @Override
    public void periodic() { // TODO ADD COMMENTS
        GutNeckStates nextSystemState = systemState;
        if (systemState == GutNeckStates.IDLE_NO_CARGO) { 
            /* Gut and neck are stationary; no cargo is stored */

            // Outputs
            commandGutSurfaceSpeed(0.0);
            commandNeckSurfaceSpeed(0.0);

            // State Transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestIntakeLeft) {
                nextSystemState = GutNeckStates.INTAKE_LEFT_NO_CARGO;
            } else if (requestIntakeRight) {
                nextSystemState = GutNeckStates.INTAKE_RIGHT_NO_CARGO;
            } 
        } else if (systemState == GutNeckStates.IDLE_ONE_CARGO) {
            /* Gut and neck are stationary; one cargo is stored in the neck */

            // Outputs
            commandGutSurfaceSpeed(0.0);
            commandNeckSurfaceSpeed(0.0);

            // State Transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot && RobotContainer.shooter.getSystemState() == ShooterState.AT_SETPOINT) {
                beginShootingSequence();
                nextSystemState = GutNeckStates.SHOOT_CARGO;
            } else if (requestIntakeLeft && !requestShoot) {
                nextSystemState = GutNeckStates.INTAKE_LEFT_ONE_CARGO;
            } else if (requestIntakeRight && !requestShoot) {
                nextSystemState = GutNeckStates.INTAKE_RIGHT_ONE_CARGO;
            }   
        } else if (systemState == GutNeckStates.IDLE_TWO_CARGO) {
            /* Gut and neck are stationary; two cargo are stored in the neck */
            
            // Outputs
            commandGutSurfaceSpeed(0.0);
            commandNeckSurfaceSpeed(0.0);

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot && RobotContainer.shooter.getSystemState() == ShooterState.AT_SETPOINT) {
                beginShootingSequence();
                nextSystemState = GutNeckStates.SHOOT_CARGO;
            } 
        } else if (systemState == GutNeckStates.INTAKE_LEFT_NO_CARGO) {
            /* Gut is moving in the left direction; the neck is stationary; no cargo is stored */

            // Outputs
            commandGutSurfaceSpeed(1.5);
            commandNeckSurfaceSpeed(0.0);

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (!requestIntakeLeft) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            } else if (getMiddleBeamBreakTriggered() && checkCargo(getMiddleColor())) {
                nextSystemState = GutNeckStates.STOW_ONE_CARGO_IN_NECK;
            }
        } else if (systemState == GutNeckStates.INTAKE_RIGHT_NO_CARGO) {
            /* Gut is moving in the right direction; the neck is stationary; no cargo is stored */

            // Output 
            commandGutSurfaceSpeed(-1.5);
            commandNeckSurfaceSpeed(0.0);

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (!requestIntakeRight) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            } else if (getMiddleBeamBreakTriggered() && checkCargo(getMiddleColor())) {
                nextSystemState = GutNeckStates.STOW_ONE_CARGO_IN_NECK;
            }
        } else if (systemState == GutNeckStates.INTAKE_LEFT_ONE_CARGO) {
            /* Gut is moving in the left direction; the neck is stationary; one cargo is stored in the neck */

            // Outputs
            commandGutSurfaceSpeed(1.5);
            commandNeckSurfaceSpeed(0.0);

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (!requestIntakeLeft) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (getMiddleBeamBreakTriggered() && checkCargo(getMiddleColor())) {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            }
        } else if (systemState == GutNeckStates.INTAKE_RIGHT_ONE_CARGO) {
            /* The gut is moving in the right direction; the neck is stationary; one cargo is stored in the neck */

            // Outputs
            commandGutSurfaceSpeed(-1.5);
            commandNeckSurfaceSpeed(0.0);

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (requestShoot) {
                systemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (!requestIntakeRight) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            } else if (getMiddleBeamBreakTriggered() && checkCargo(getMiddleColor())) {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            }
        } else if (systemState == GutNeckStates.STOW_ONE_CARGO_IN_NECK) {
            /* The neck is moving in the up direction; the gut is stationary; it is currently storing the first cargo */

            // Outputs
            commandGutSurfaceSpeed(0.0);
            commandNeckSurfaceSpeed(3.0);

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (getTopBeamBreakTriggered()) {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            }
        } else if (systemState == GutNeckStates.SHOOT_CARGO) {
            /* The neck is moving in the up direction; the gut is stationary; it is current shooting the balls stored in the neck */

            // Outputs
            commandGutSurfaceSpeed(0.0);
            commandNeckSurfaceSpeed(2.5);

            if (!getMiddleBeamBreakTriggered() && !getTopBeamBreakTriggered() && !ballsExpelledFromNeck) {
                ballsExpelledFromNeck = true;
                shootingTimer.start();
            }

            if (getMiddleBeamBreakTriggered() || getTopBeamBreakTriggered()) {
                ballsExpelledFromNeck = false;
                shootingTimer.stop();
                shootingTimer.reset();
            }

            // State transitions
            if (requestSpitLeft) {
                nextSystemState = GutNeckStates.SPIT_LEFT;
            } else if (requestSpitRight) {
                nextSystemState = GutNeckStates.SPIT_RIGHT;
            } else if (shootingTimer.get() > 0.5 && ballsExpelledFromNeck) {
                exitShootingSequence();
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        } else if (systemState == GutNeckStates.SPIT_LEFT) {
            /* The neck is moving in the down direction; the gut is moving in the left direction; there may or may not be balls somewhere in the robot */

            // Outputs
            commandGutSurfaceSpeed(-2.0);
            commandNeckSurfaceSpeed(-1.0);

            // State transitions
            if (!requestSpitLeft) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        } else if (systemState == GutNeckStates.SPIT_RIGHT) {
            /* The neck is moving in the down direction; the gut is moving in the right direction; there may or may not be balls somewhere in the robot */

            // Outputs
            commandGutSurfaceSpeed(2.0);
            commandNeckSurfaceSpeed(-1.0);

            // State transitions
            if (!requestSpitRight) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        }
        systemState = nextSystemState;
        SmartDashboard.putString("GutNeck State", getSystemState().name());
        SmartDashboard.putBoolean("GutNeck Request Shoot", requestShoot);
        SmartDashboard.putBoolean("Middle BeamBreak", getMiddleBeamBreakTriggered());
        SmartDashboard.putBoolean("Top BeamBreak", getTopBeamBreakTriggered());
    }

    // Private method to begin the shooting sequence
    private void beginShootingSequence() {
        shootingTimer.reset();
        ballsExpelledFromNeck = false;
    }

    // Private method to exit the shooting sequence
    private void exitShootingSequence() {
        shootingTimer.reset();
        shootingTimer.stop();
    }

    // Private method to check whether or not to accept a given cargo
    private boolean checkCargo(BallColor color) { // TODO check what happens if you unplug the sensor
        if (acceptOpposingCargo) {
            return color != BallColor.NONE;
        } else {
            return color == Robot.allianceColor;
        }
    }
    
}