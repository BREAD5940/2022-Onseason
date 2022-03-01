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

    DoubleSolenoid climberSolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CLIMBER_FORWARD_CHANNEL, CLIMBER_REVERSE_CHANNEL);
    TalonFX topMotor = new TalonFX(TOP_CLIMBER_MOTOR_ID);
    TalonFX bottomMotor = new TalonFX(BOTTOM_CLIMBER_MOTOR_ID);

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

    // public void setPosition(double meters) {
    //     double output = metersToIntegratedSensorUnits(MathUtil.clamp(meters, CLIMBER_MINIMUM_TRAVEL, CLIMBER_MAXIMUM_TRAVEL));
    //     topMotor.set(ControlMode.MotionMagic, output);
    // }

    public void setPercent(double percent) {
        topMotor.set(ControlMode.PercentOutput, -percent);
    }

    public void neutral() {
        topMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void extend() {
        climberSolenoids.set(Value.kForward);
    }

    public void retract() {
        climberSolenoids.set(Value.kReverse);
    }

    public double getPositionMeters() {
        return integratedSensorUnitsToMeters(topMotor.getSelectedSensorPosition());
    }

    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER)/2048.0);
    }

    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER));
    }

    private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * (600.0/2048.0) * Math.PI * CLIMBER_PITCH_DIAMETER)/60.0);
    }

    private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
        return metersPerSecond * (60.0/(CLIMBER_GEARING * (600.0/2048.0) * Math.PI * CLIMBER_PITCH_DIAMETER));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPositionMeters());
    }
    
}
