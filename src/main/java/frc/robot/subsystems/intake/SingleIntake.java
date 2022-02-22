package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class SingleIntake {

    private final TalonFX motor;

    public SingleIntake(int motorChannel, boolean isReversed) {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 25.0, 25.0, 1.5);
        motor = new TalonFX(motorChannel);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(isReversed);
        motor.configAllSettings(motorConfig);
    }

    public void set(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
    
}
