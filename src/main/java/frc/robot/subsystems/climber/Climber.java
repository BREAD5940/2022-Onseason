package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Climber.*;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid climberSolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CLIMBER_FORWARD_CHANNEL, CLIMBER_REVERSE_CHANNEL);
    private final TalonFX topMotor = new TalonFX(TOP_CLIMBER_MOTOR_ID);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_CLIMBER_MOTOR_ID);

    // State variables
    private ClimberStates systemState = ClimberStates.NEUTRAL_TILTED_IN;
    private double climberSetpoint = CLIMBER_MINIMUM_TRAVEL;
    private boolean extended = false;

    public Climber() {

        // Configure the top climber motor
        TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();
        topMotorConfig.slot0.kP = integratedSensorUnitsToMetersPerSecond(1.0) * 1023.0;
        topMotorConfig.slot0.kI = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        topMotorConfig.slot0.kD = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        topMotorConfig.slot0.kF = 1023.0/metersPerSecondToIntegratedSensorUnits(MAX_CLIMBER_TRAVEL_SPEED);
        topMotorConfig.motionCruiseVelocity = metersPerSecondToIntegratedSensorUnits(1.4);
        topMotorConfig.motionAcceleration = metersPerSecondToIntegratedSensorUnits(10.0);
        topMotor.setInverted(TOP_CLIMBER_MOTOR_INVERT_TYPE);
        topMotor.setNeutralMode(NeutralMode.Brake);
        topMotor.configAllSettings(topMotorConfig);
        topMotor.setSelectedSensorPosition(0.0);

        // Configure the bottom climber motor 
        TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();
        bottomMotor.setInverted(BOTTOM_CLIMBER_MOTOR_INVERT_TYPE);
        bottomMotor.configAllSettings(bottomMotorConfig);
        bottomMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.follow(topMotor);
    }

    // Commands the climber to a certain setpoint
    private void commandClimberSetpoint(double meters) {
        double output = metersToIntegratedSensorUnits(MathUtil.clamp(meters, CLIMBER_MINIMUM_TRAVEL, CLIMBER_MAXIMUM_TRAVEL));
        topMotor.set(ControlMode.MotionMagic, output);
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
    private void requestMidRungHeight() {
        // systemState = ClimberStates.MID_RUNG_HEIGHT;
    }   

    // Returns the position of the climber in meters
    public double getPositionMeters() {
        return integratedSensorUnitsToMeters(topMotor.getSelectedSensorPosition());
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
        NEUTRAL_TILTED_IN,
        RETRACTED_TILTED_IN,
        MID_RUNG_HEIGHT_TILTED_IN,

    }

    @Override
    public void periodic() {
        if (systemState == ClimberStates.NEUTRAL_TILTED_IN) {
            commandNeutral();
            if (extended) {
                commandSolenoidsReversed();
                extended = false;
            } 
        } else if (systemState == ClimberStates.RETRACTED_TILTED_IN) {
            commandClimberSetpoint(CLIMBER_MINIMUM_TRAVEL);
            if (extended) {
                commandSolenoidsReversed();
                extended = false;
            }
        } else if (systemState == ClimberStates.MID_RUNG_HEIGHT_TILTED_IN) {
            commandClimberSetpoint(CLIMBER_MID_RUNG_HEIGHT);
            if (extended) {
                commandSolenoidsReversed();
                extended = false;
            }
        }
    }
    
}
