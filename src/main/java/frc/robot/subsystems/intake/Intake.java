package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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


    public Intake(int motorID, TalonFXInvertType invertType, int pneumaticsForwardChannel, int pneumaticsReverseChannel, int pneumaticsModuleNumber) {
        // Configure the intake motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor = TalonFXFactory.createDefaultTalon(motorID);
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 25.0, 25.0, 1.5);
        motor.setInverted(invertType);
        motor.setNeutralMode(NeutralMode.Coast);

        // Configure the double solenoid
        doubleSolenoids = new DoubleSolenoid(pneumaticsModuleNumber, PneumaticsModuleType.CTREPCM, pneumaticsForwardChannel, pneumaticsReverseChannel);
    }

    // Requests the intake to extend and suck
    public void requestIntake() {
        systemState = IntakeState.SUCK_EXTENDED;
    }

    // Requests the intake to retract and spit
    public void requestOuttake() {
        systemState = IntakeState.SPIT_RETRACTED;
    }

    // Requests the intake to go into idle mode
    public void requestIdle() {
        systemState = IntakeState.IDLE_RETRACTED;
    }

    public enum IntakeState {
        IDLE_RETRACTED, 
        SUCK_EXTENDED,
        SPIT_RETRACTED, 
    }
    
    // Update the statemachine in the periodic method of the intake subsystem
    @Override
    public void periodic() {
        if (systemState == IntakeState.IDLE_RETRACTED) {
            motor.set(ControlMode.PercentOutput, 0.0);
            doubleSolenoids.set(Value.kReverse);
        } else if (systemState == IntakeState.SUCK_EXTENDED) {
            motor.set(ControlMode.PercentOutput, 0.8);
            doubleSolenoids.set(Value.kForward);
        } else if (systemState == IntakeState.SPIT_RETRACTED) {
            motor.set(ControlMode.PercentOutput, -0.8);
            doubleSolenoids.set(Value.kReverse); 
        }
    }
    
}
