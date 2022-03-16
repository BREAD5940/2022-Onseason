package frc.robot.subsystems.statemachines;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.TalonFXFactory;

public class Intake extends SubsystemBase {

    // Intake Hardware
    private final TalonFX motor; 
    private final DoubleSolenoid doubleSolenoids;

    // Variables to track system state
    private IntakeState systemState = IntakeState.IDLE_RETRACTED;
    private boolean spit = false;
    private boolean extended = true;


    public Intake(int motorID, TalonFXInvertType invertType, int pneumaticsForwardChannel, int pneumaticsReverseChannel, int pneumaticsModuleNumber) {
        // Configure the intake motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor = TalonFXFactory.createDefaultTalon(motorID);
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 25.0, 25.0, 1.5);
        motor.setInverted(invertType);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        // Configure the double solenoid
        doubleSolenoids = new DoubleSolenoid(pneumaticsModuleNumber, PneumaticsModuleType.CTREPCM, pneumaticsForwardChannel, pneumaticsReverseChannel);
    }

    // Requests the intake to extend and suck; value between [0, 1]
    public void requestIntake() {
        systemState = IntakeState.SUCK_EXTENDED;
    }

    // Requests the intake to retract and spit; value between [0, 1]
    public void requestOuttakeRetracted(boolean spit) {
        this.spit = spit;
        systemState = IntakeState.SPIT_RETRACTED;
    }

    // Requests the intake the extend and outtake; value between [0, 1]
    public void requestOuttakeExtended(boolean spit) {
        this.spit = spit;
        systemState = IntakeState.SPIT_EXTENDED;
    }

    // Requests the intake to go into idle mode retracted
    public void requestIdleRetracted() {
        systemState = IntakeState.IDLE_RETRACTED;
    }

    // Requests the intake to go into idle mode extended
    public void requestIdleExtended() {
        systemState = IntakeState.IDLE_EXTENDED;
    }

    public enum IntakeState {
        IDLE_RETRACTED, 
        IDLE_EXTENDED,
        SUCK_EXTENDED,
        SPIT_RETRACTED, 
        SPIT_EXTENDED
    }
    
    // Update the statemachine in the periodic method of the intake subsystem
    @Override
    public void periodic() {
        if (systemState == IntakeState.IDLE_RETRACTED) {
            motor.set(ControlMode.PercentOutput, 0.0);
            if (extended) {
                doubleSolenoids.set(Value.kReverse);
                extended = false;
            }
        } else if (systemState == IntakeState.IDLE_EXTENDED) {
            motor.set(ControlMode.PercentOutput, 0.0);
            if (!extended) {
                doubleSolenoids.set(Value.kForward);
                extended = true;
            }
        } else if (systemState == IntakeState.SUCK_EXTENDED) {
            motor.set(ControlMode.PercentOutput, 0.8);
            if (!extended) {
                doubleSolenoids.set(Value.kForward);
                extended = true;
            }
        } else if (systemState == IntakeState.SPIT_RETRACTED) {
            motor.set(ControlMode.PercentOutput, spit ? -0.3 : -0.8);
            if (extended) {
                doubleSolenoids.set(Value.kReverse); 
                extended = false;
            }
        } else if (systemState == IntakeState.SPIT_EXTENDED) {
            motor.set(ControlMode.PercentOutput, spit ? -0.3 : -0.8);
            if (!extended) {
                doubleSolenoids.set(Value.kForward);
                extended = true;
            }
        } else {
            System.out.println("Error: Intake statemachine is not working properly.");
        }
    }
    
}
