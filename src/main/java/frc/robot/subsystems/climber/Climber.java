package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import frc.robot.drivers.TalonUtil;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import static frc.robot.Constants.Climber.*;

import java.io.ObjectInputFilter.Status;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid climberSolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CLIMBER_FORWARD_CHANNEL, CLIMBER_REVERSE_CHANNEL);
    private final TalonFX topMotor = new TalonFX(TOP_CLIMBER_MOTOR_ID);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_CLIMBER_MOTOR_ID);

    // State variables
    private ClimberStates systemState = ClimberStates.STARTING_CONFIGURATION;
    private double climberSetpoint = -1.0;
    private boolean extended = true;
    private boolean requestNextState = false;
    private boolean requestPreviousState = false;
    private double lastTransitionedFPGASeconds = 0.0;

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
        topMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        topMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        TalonUtil.checkError(topMotor.configAllSettings(topMotorConfig), "Top Climber Motor Configuration Failed");
        topMotor.setSelectedSensorPosition(0.0);

        // Configure the bottom climber motor 
        TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();
        bottomMotor.setInverted(BOTTOM_CLIMBER_MOTOR_INVERT_TYPE);
        TalonUtil.checkError(bottomMotor.configAllSettings(bottomMotorConfig), "Bottom Climber Motor Configuration Failed");
        bottomMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.follow(topMotor);
        topMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 197);
        topMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 193);
    }

    // Commands the max velocity of the climber
    private void commandMaxVelocity(double velocity) {
        topMotor.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(velocity));
    }

    public void commandPercent(double percent) {
        topMotor.set(ControlMode.PercentOutput, percent);
    }

    // Commands the height setpoint of the climber
    public void commandHeightSetpoint(double meters, boolean isLifting) {
        double output = metersToIntegratedSensorUnits(MathUtil.clamp(meters, CLIMBER_MINIMUM_TRAVEL + 0.01, CLIMBER_MAXIMUM_TRAVEL - 0.01));
        topMotor.set(ControlMode.MotionMagic, output, DemandType.ArbitraryFeedForward, isLifting ? -0.16893148154 : 0.0);
    }

    // Commands the neutral mode of the climber
    public void commandNeutralMode(boolean set) {
        topMotor.setNeutralMode(set ? NeutralMode.Coast : NeutralMode.Brake);
        bottomMotor.setNeutralMode(set ? NeutralMode.Coast : NeutralMode.Brake);
    }

    // Commands the climber solenoids forward
    public void commandSolenoidsForward() {
        climberSolenoids.set(Value.kForward);
    }

    // Commands the climber solenoids backward
    public void commandSolenoidsReversed() {
        climberSolenoids.set(Value.kReverse);
    }

    // Returns the position of the climber in meters
    public double getPositionMeters() {
        return integratedSensorUnitsToMeters(topMotor.getSelectedSensorPosition());
    }

    // Returns the system state of the climber
    public ClimberStates getSystemState() {
        return systemState;
    }

    // Sends a request for the climber to go to the next state
    public void requestNextState() {
        requestNextState = true;
    } 

    // Sends a request for the climber to go to the previous state
    public void requestPreviousState() {
        requestPreviousState = true;
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
        STARTING_CONFIGURATION,
        READY_FOR_MID_RUNG,
        CLIMB_TO_MID_RUNG,
        EXTENDED_BELOW_HIGH_RUNG,
        READY_FOR_HIGH_RUNG,
        LATCHED_TO_HIGH_RUNG,
        CLIMB_TO_HIGH_RUNG,
        EXTEND_TO_TRAVERSAL_RUNG,   
        CLIMB_TO_TRAVERSAL
    }

    @Override
    public void periodic() {
        // ClimberStates nextSystemState = systemState;
        // if (systemState == ClimberStates.STARTING_CONFIGURATION) {
        //     // Outputs
        //     commandHeightSetpoint(CLIMBER_RETRACTED_HEIGHT, false);
        //     handleSolenoidExtension(false);

        //     // State Transitions
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.READY_FOR_MID_RUNG;
        //         requestNextState = false;
        //     } else if (requestPreviousState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         requestPreviousState = false;
        //     }
        // } else if (systemState == ClimberStates.READY_FOR_MID_RUNG) {
        //     // Outputs
        //     commandHeightSetpoint(CLIMBER_MID_RUNG_HEIGHT, false);
        //     handleSolenoidExtension(false);

        //     // State transitions
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.CLIMB_TO_MID_RUNG;
        //         requestNextState = false;
        //     } else if (requestPreviousState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.STARTING_CONFIGURATION;
        //         requestPreviousState = false;
        //     }
        // } else if (systemState == ClimberStates.CLIMB_TO_MID_RUNG) {
        //     // Outputs
        //     if (BreadUtil.getFPGATimeSeconds() - lastTransitionedFPGASeconds < 0.4) {
        //         commandHeightSetpoint(CLIMBER_RETRACTED_HEIGHT, true);
        //     } else {
        //         commandHeightSetpoint(CLIMBER_HEIGHT_BEFORE_NEXT_RUNG, false);
        //     }
        //     handleSolenoidExtension(false);

        //     // State transitions
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.EXTENDED_BELOW_HIGH_RUNG;
        //         requestNextState = false;
        //     } else if (requestPreviousState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.READY_FOR_MID_RUNG;
        //         requestPreviousState = false;
        //     }
        // } else if (systemState == ClimberStates.EXTENDED_BELOW_HIGH_RUNG) {
        //     // Outputs
        //     commandHeightSetpoint(CLIMBER_HEIGHT_BEFORE_NEXT_RUNG, false);
        //     handleSolenoidExtension(true);

        //     // State Transitoins
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.READY_FOR_HIGH_RUNG;
        //         requestNextState = false;   
        //     } else if (requestPreviousState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.CLIMB_TO_MID_RUNG;
        //         requestPreviousState = false;
        //     }
        // } else if (systemState == ClimberStates.READY_FOR_HIGH_RUNG) {
        //     // Outputs
        //     commandHeightSetpoint(CLIMBER_HEIGHT_TRANSITIONING_TO_NEXT_RUNG, false);
        //     handleSolenoidExtension(true);

        //     // State Transitions
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.LATCHED_TO_HIGH_RUNG;
        //         requestNextState = false;
        //     } else if (requestPreviousState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.EXTENDED_BELOW_HIGH_RUNG;
        //         requestPreviousState = false;
        //     }
        // } else if (systemState == ClimberStates.LATCHED_TO_HIGH_RUNG) {
        //     // Outputs
        //     commandHeightSetpoint(CLIMBER_HEIGHT_TRANSITIONING_TO_NEXT_RUNG, false);
        //     handleSolenoidExtension(false);
                
        //     // State Transitions
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.CLIMB_TO_HIGH_RUNG;
        //         requestNextState = false;
        //     } else if (requestPreviousState) {
        //         lastTransitionedFPGASeconds = BreadUtil.getFPGATimeSeconds();
        //         nextSystemState = ClimberStates.READY_FOR_HIGH_RUNG;
        //         requestPreviousState = false;
        //     }
        // } else if (systemState == ClimberStates.CLIMB_TO_HIGH_RUNG) {
        //     // Outputs
        //     if (requestNextState) {
        //         commandHeightSetpoint(CLIMBER_RETRACTED_HEIGHT, false);
        //         handleSolenoidExtension(false);
        //     } else if (requestPreviousState) {
        //         commandHeightSetpoint(CLIMBER_HEIGHT_BEFORE_NEXT_RUNG, false);
        //         handleSolenoidExtension(false);
        //     }

        //     // States Transitions
        //     if (requestNextState) {
        //         lastTransitionedFPGASeconds=  
        //         nextSystemState = ClimberStates.EXTEND_TO_TRAVERSAL_RUNG;

        //     }
        SmartDashboard.putNumber("Climber Height", getPositionMeters());
    } 

    private void handleSolenoidExtension(boolean wantsExtended) {
        if (wantsExtended && !extended) {
            commandSolenoidsForward();
            extended = true;
        } else if (!wantsExtended && extended) {
            commandSolenoidsReversed();
            extended = false;
        }
    }
    
}
