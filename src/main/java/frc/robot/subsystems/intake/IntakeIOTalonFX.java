package frc.robot.subsystems.intake;

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
import frc.robot.drivers.TalonFXFactory;
import frc.robot.drivers.TalonUtil;

public class IntakeIOTalonFX implements IntakeIO {

    private final DoubleSolenoid solenoids;
    
    private final TalonFX motor;

    boolean solenoidExtended = false;

    public IntakeIOTalonFX(int motorID, TalonFXInvertType invertType, int pneumaticsForwardChannel, int pneumaticsReverseChannel, int pneumaticsModuleNumber) {
        solenoids = new DoubleSolenoid(pneumaticsModuleNumber, PneumaticsModuleType.CTREPCM, pneumaticsForwardChannel, pneumaticsReverseChannel);
        motor = TalonFXFactory.createDefaultTalon(motorID);

        
        /* configurations for the intake motor */
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        motor.setInverted(invertType);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setStatusFramePeriod(StatusFrame.Status_1_General, 229);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 227);
        TalonUtil.checkError(motor.configAllSettings(config), "Intake Configuration with ID " + motorID + " Failed");
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.solenoidsForward = solenoids.get() == Value.kForward;
        inputs.appliedVoltage = motor.getMotorOutputVoltage();
        inputs.currentAmps = motor.getSupplyCurrent();
        inputs.tempCelcius = motor.getTemperature();
    }

    @Override
    public void spin(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void extend(boolean wantsIntakeExtended) {
        if (solenoidExtended&&wantsIntakeExtended) { // It's reversed because when the solenoids are extended, the intake is retracted. 
            solenoids.set(Value.kReverse);
            solenoidExtended = false;
        } else if (!solenoidExtended&&!wantsIntakeExtended) {
            solenoids.set(Value.kForward);
            solenoidExtended = true;
        }
    }
    
}
