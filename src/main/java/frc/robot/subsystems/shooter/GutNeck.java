package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.sensors.BeamBreak;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensor.BallColor;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

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

    // Statemachine inputs
    private boolean requestIntakeLeft = false;
    private boolean requestIntakeRight = false;
    private boolean requestShoot = false;

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
    }

    // Returns the output of the left beam break
    public boolean getLeftBeamBreak() {
        return leftBeamBreak.get();
    }

    // Returns the output of the right beam break 
    public boolean getRightBeamBreak() {
        return rightBeamBreak.get();
    }

    // Returns the output of the middle beam break
    public boolean getMiddleBeamBreak() {
        return middleBeamBreak.get();
    }

    // Returns the output of the top beam break
    public boolean getTopBeamBreak() {
        return topBeamBreak.get();
    }

    // Returns the output of the color sensors
    public BallColor getColorSensor() {
        return colorSensor.get();
    }

    // Returns the surface speed of the gut
    private double getGutSurfaceSpeed() {
        return integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(gutMotor.getSelectedSensorVelocity());
    }

    // Returns the surface speed of the neck
    private double getNeckSurfaceSpeed() {
        return integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(neckMotor.getSelectedSensorVelocity());
    }

    // Sets the surface speed of the gut
    private void setGutSurfaceSpeed(double speed) {
        gutMotor.set(ControlMode.Velocity, gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speed));
    }

    // Sets the surface speed of the neck
    private void setNeckSurfaceSpeed(double speed) {
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
        SHOOT_ONE_CARGO,
        SHOOT_TWO_CARGO
    } 

    // Update the state machine
    @Override
    public void periodic() { // TODO ADD COMMENTS
        GutNeckStates nextSystemState = null;
        if (systemState == GutNeckStates.IDLE_NO_CARGO) {
            // Outputs
            setGutSurfaceSpeed(0.0);
            setNeckSurfaceSpeed(0.0);

            // State Transitions
            if (requestIntakeLeft) {
                nextSystemState = GutNeckStates.INTAKE_LEFT_NO_CARGO;
            } else if (requestIntakeRight) {
                nextSystemState = GutNeckStates.INTAKE_RIGHT_NO_CARGO;
            } else if (requestShoot) {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            } else {
                nextSystemState = GutNeckStates.IDLE_NO_CARGO;
            }
        } else if (systemState == GutNeckStates.IDLE_ONE_CARGO) {
            // Outputs
            setGutSurfaceSpeed(0.0);
            setNeckSurfaceSpeed(0.0);

            // State Transitions
            if (requestIntakeLeft) {
                nextSystemState = GutNeckStates.INTAKE_LEFT_ONE_CARGO;
            } else if (requestIntakeRight) {
                nextSystemState = GutNeckStates.INTAKE_RIGHT_ONE_CARGO;
            // } else if (requestShoot&&RobotContainer.shooter.getSystemState()==ShooterState.AT_SETPOINT) {
            //     nextSystemState = GutNeckStates.SHOOT_ONE_CARGO;
            } else {
                nextSystemState = GutNeckStates.IDLE_ONE_CARGO;
            }       
        } else if (systemState == GutNeckStates.IDLE_TWO_CARGO) {
            // Outputs
            setGutSurfaceSpeed(0.0);
            setNeckSurfaceSpeed(0.0);

            // State transitions
            if (requestIntakeLeft) {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            } else if (requestIntakeRight) {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            // } else if (requestShoot&&RobotContainer.shooter.getSystemState()==ShooterState.AT_SETPOINT) {
            //     nextSystemState = GutNeckStates.SHOOT_TWO_CARGO;
            } else {
                nextSystemState = GutNeckStates.IDLE_TWO_CARGO;
            }
        } else if (systemState == GutNeckStates.INTAKE_LEFT_NO_CARGO) {
            // Outputs
            setGutSurfaceSpeed(1.5);
            setNeckSurfaceSpeed(0.0);

            // State transitions


        }
        systemState = nextSystemState;
    }
    
}
