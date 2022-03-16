package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Climber.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid climberSolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CLIMBER_FORWARD_CHANNEL, CLIMBER_REVERSE_CHANNEL);
    private final TalonFX topMotor = new TalonFX(TOP_CLIMBER_MOTOR_ID);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_CLIMBER_MOTOR_ID);

    // State variables
    private ClimberStates systemState = ClimberStates.NEUTRAL;
    private double climberSetpoint = -1.0;
    private boolean extended = true;
    private boolean wantsExtended = false;
    private boolean wantsLift = false;

    public Climber() {

        // Configure the top climber motor
        TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();
        topMotorConfig.slot0.kP = integratedSensorUnitsToMetersPerSecond(1.0) * 1023.0;
        topMotorConfig.slot0.kI = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        topMotorConfig.slot0.kD = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        topMotorConfig.slot0.kF = 1023.0/metersPerSecondToIntegratedSensorUnits(MAX_CLIMBER_TRAVEL_SPEED);
        topMotorConfig.motionCruiseVelocity = metersPerSecondToIntegratedSensorUnits(1.4);
        topMotorConfig.motionAcceleration = metersPerSecondToIntegratedSensorUnits(5.0);
        topMotorConfig.voltageCompSaturation = 10.5;
        topMotor.setInverted(TOP_CLIMBER_MOTOR_INVERT_TYPE);
        topMotor.setNeutralMode(NeutralMode.Brake);
        topMotor.enableVoltageCompensation(true);
        topMotor.configAllSettings(topMotorConfig);
        topMotor.setSelectedSensorPosition(0.0);

        // Configure the bottom climber motor 
        TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();
        bottomMotor.setInverted(BOTTOM_CLIMBER_MOTOR_INVERT_TYPE);
        bottomMotor.configAllSettings(bottomMotorConfig);
        bottomMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.follow(topMotor);
    }

    // Commands the max velocity of the climber
    private void commandMaxVelocity(double velocity) {
        topMotor.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(velocity));
    }

    // Commands the height setpoint of the climber
    private void commandHeightSetpoint(double meters, boolean isLifting) {
        double output = metersToIntegratedSensorUnits(MathUtil.clamp(meters, CLIMBER_MINIMUM_TRAVEL + 0.01, CLIMBER_MAXIMUM_TRAVEL - 0.01));
        topMotor.set(ControlMode.MotionMagic, output, DemandType.ArbitraryFeedForward, isLifting ? -0.16893148154 : 0.0);
    }

    // Commands the neutral mode of the climber
    public void commandNeutralMode(boolean set) {
        topMotor.setNeutralMode(set ? NeutralMode.Coast : NeutralMode.Brake);
        bottomMotor.setNeutralMode(set ? NeutralMode.Coast : NeutralMode.Brake);
    }

    // Commands the climber to neutral
    private void commandNeutral() {
        topMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // Commands the climber solenoids forward
    private void commandSolenoidsForward() {
        climberSolenoids.set(Value.kForward);
    }

    // Commands the climber solenoids backward
    private void commandSolenoidsReversed() {
        climberSolenoids.set(Value.kReverse);
    }

    // Requests the climber to go to the mid rung height
    public void requestMidRungHeight(boolean wantsExtended, boolean wantsLift, double maxVelocity) {
        commandMaxVelocity(maxVelocity);
        this.wantsExtended = wantsExtended;
        this.wantsLift = wantsLift;
        systemState = ClimberStates.MID_RUNG_HEIGHT;
    }   

    // Requests the climber to go to the ready for next rung height
    public void requestReadyForNextRung(boolean wantsExtended, boolean wantsLift, double maxVelocity) {
        commandMaxVelocity(maxVelocity);
        this.wantsExtended = wantsExtended;
        this.wantsLift = wantsLift;
        systemState = ClimberStates.READY_FOR_NEXT_RUNG_HEIGHT;
    }

    // Requests the climber to go the pull off height
    public void requestPullOffHeight(boolean wantsExtended, boolean wantsLift, double maxVelocity) {
        commandMaxVelocity(maxVelocity);
        this.wantsExtended = wantsExtended;
        this.wantsLift = wantsLift;
        systemState = ClimberStates.PULLED_OFF;
    }

    // Requests the climber to go to the before next rung height
    public void requestHeightBeforeNextRung(boolean wantsExtended, boolean wantsLift, double maxVelocity) {
        commandMaxVelocity(maxVelocity);
        this.wantsExtended = wantsExtended;
        this.wantsLift = wantsLift;
        systemState = ClimberStates.BEFORE_NEXT_RUNG_HEIGHT;
    }

    // Requests the climber to go to the transitioning rung height
    public void requestHeightToTransitionToNextRung(boolean wantsExtended, boolean wantsLift, double maxVelocity) {
        commandMaxVelocity(maxVelocity);
        this.wantsExtended = wantsExtended;
        this.wantsLift = wantsLift;
        systemState = ClimberStates.TRANSITIONING_TO_NEXT_RUNG;
    }

    // Requests the climber to go to its fully retracted height
    public void requestRetracted(boolean wantsExtended, boolean wantsLift, double maxVelocity) {
        commandMaxVelocity(maxVelocity);
        this.wantsExtended = wantsExtended;
        this.wantsLift = wantsLift;
        systemState = ClimberStates.RETRACTED;
    }

    // Returns the position of the climber in meters
    public double getPositionMeters() {
        return integratedSensorUnitsToMeters(topMotor.getSelectedSensorPosition());
    }

    // Returns the system state of the climber
    public ClimberStates getSystemState() {
        return systemState;
    }

    // Converted integrated sensor units to meters
    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER)/2048.0);
    }

    // Converts meters to integrated sensor units
    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER));
    }

    // Converts integrated sensor units to meters per second
    private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * (600.0/2048.0) * Math.PI * CLIMBER_PITCH_DIAMETER)/60.0);
    }

    // Converts meters per second to integrated sensor units
    private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
        return metersPerSecond * (60.0/(CLIMBER_GEARING * (600.0/2048.0) * Math.PI * CLIMBER_PITCH_DIAMETER));
    }

    // Defines all of the climber states
    public enum ClimberStates { 
        NEUTRAL,
        RETRACTED,
        MID_RUNG_HEIGHT,
        READY_FOR_NEXT_RUNG_HEIGHT,
        BEFORE_NEXT_RUNG_HEIGHT,
        TRANSITIONING_TO_NEXT_RUNG,
        PULLED_OFF
    }

    @Override
    public void periodic() {
        ClimberStates nextSystemState = systemState;
        if (systemState == ClimberStates.NEUTRAL) {
            // Outputs
            commandNeutral();
            handleSolenoidExtension();
            climberSetpoint = -1.0;
            
        } else if (systemState == ClimberStates.RETRACTED) {
            // Outputs
            commandHeightSetpoint(CLIMBER_RETRACTED_HEIGHT, wantsLift);
            handleSolenoidExtension();
            climberSetpoint = CLIMBER_RETRACTED_HEIGHT;
            
            // State Transitions
            if (BreadUtil.atReference(getPositionMeters(), CLIMBER_RETRACTED_HEIGHT, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextSystemState = ClimberStates.NEUTRAL;
            }
        } else if (systemState == ClimberStates.MID_RUNG_HEIGHT) {
            // Outputs
            commandHeightSetpoint(CLIMBER_MID_RUNG_HEIGHT, wantsLift);
            handleSolenoidExtension();
            climberSetpoint = CLIMBER_MID_RUNG_HEIGHT;

            // State Transitions
            if (BreadUtil.atReference(getPositionMeters(), CLIMBER_MID_RUNG_HEIGHT, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextSystemState = ClimberStates.NEUTRAL;
            }
            
        } else if (systemState == ClimberStates.READY_FOR_NEXT_RUNG_HEIGHT) {
            // Outputs
            commandHeightSetpoint(CLIMBER_READY_FOR_NEXT_RUNG_HEIGHT, wantsLift);
            handleSolenoidExtension();
            climberSetpoint = CLIMBER_READY_FOR_NEXT_RUNG_HEIGHT;

            // State Transitions
            if (BreadUtil.atReference(getPositionMeters(), CLIMBER_READY_FOR_NEXT_RUNG_HEIGHT, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextSystemState = ClimberStates.NEUTRAL;
            }
        } else if (systemState == ClimberStates.BEFORE_NEXT_RUNG_HEIGHT) {
            // Outputs
            commandHeightSetpoint(CLIMBER_HEIGHT_BEFORE_NEXT_RUNG, wantsLift);
            handleSolenoidExtension();
            climberSetpoint = CLIMBER_HEIGHT_BEFORE_NEXT_RUNG;
            
            // State Transitions
            if (BreadUtil.atReference(getPositionMeters(), CLIMBER_HEIGHT_BEFORE_NEXT_RUNG, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextSystemState  = ClimberStates.NEUTRAL;
            }
        } else if (systemState == ClimberStates.TRANSITIONING_TO_NEXT_RUNG) {
            // Outputs
            commandHeightSetpoint(CLIMBER_HEIGHT_TRANSITIONING_TO_NEXT_RUNG, wantsLift);
            handleSolenoidExtension();
            climberSetpoint = CLIMBER_HEIGHT_TRANSITIONING_TO_NEXT_RUNG;
            
            // State Transitions
            if (BreadUtil.atReference(getPositionMeters(), CLIMBER_HEIGHT_TRANSITIONING_TO_NEXT_RUNG, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextSystemState  = ClimberStates.NEUTRAL;
            }
        } else if (systemState == ClimberStates.PULLED_OFF) {
            // Outputs
            commandHeightSetpoint(CLIMBER_HEIGHT_PULLED_OFF, wantsLift);
            handleSolenoidExtension();
            climberSetpoint = CLIMBER_HEIGHT_PULLED_OFF;
            
            // State Transitions
            if (BreadUtil.atReference(getPositionMeters(), CLIMBER_HEIGHT_PULLED_OFF, CLIMBER_SETPOINT_TOLERANCE, true)) {
                nextSystemState  = ClimberStates.NEUTRAL;
            }
        }
        systemState = nextSystemState;
        SmartDashboard.putString("Climber State", systemState.name());
        SmartDashboard.putNumber("Climber Setpoint", climberSetpoint);
        SmartDashboard.putNumber("Climber Height", getPositionMeters());
    }

    private void handleSolenoidExtension() {
        if (wantsExtended && !extended) {
            commandSolenoidsForward();
            extended = true;
        } else if (!wantsExtended && extended) {
            commandSolenoidsReversed();
            extended = false;
        }
    }
    
}
