package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Flywheel.*;

public class Flywheel extends SubsystemBase {

    private final TalonFX leftMotor = new TalonFX(LEFT_MOTOR_ID); 
    private final TalonFX rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    private double lastSetpoint = 0.0;

    public Flywheel() {
        // Configure left motor
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.slot0.kP = integratedSensorUnitsToFlywheelRPM(0.0015) * 1023.0;
        leftMotorConfig.slot0.kI = integratedSensorUnitsToFlywheelRPM(0.0) * 1023.0;
        leftMotorConfig.slot0.kD = integratedSensorUnitsToFlywheelRPM(0.0) * 1023.0;
        leftMotorConfig.slot0.kF = 0.0;
        leftMotorConfig.slot0.closedLoopPeakOutput = 1.0;
        leftMotorConfig.peakOutputForward = 1.0;
        leftMotorConfig.peakOutputReverse = 0.0;
        leftMotorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;
        leftMotorConfig.velocityMeasurementWindow = 16;
        leftMotorConfig.voltageCompSaturation = 11.5;
        leftMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80, 80, 1.5);
        leftMotor.configAllSettings(leftMotorConfig);
        leftMotor.selectProfileSlot(0, 0);
        leftMotor.setInverted(LEFT_MOTOR_DIRECTION);
        leftMotor.enableVoltageCompensation(true);

        // Configure right motor
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.peakOutputForward = 1.0;
        rightMotorConfig.peakOutputReverse = -0.0;
        rightMotorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
        rightMotorConfig.velocityMeasurementWindow = 8;
        rightMotorConfig.voltageCompSaturation = 11.0;
        rightMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80, 80, 1.5);
        rightMotor.configAllSettings(rightMotorConfig);
        rightMotor.setInverted(RIGHT_MOTOR_DIRECTION);
        rightMotor.follow(leftMotor);
        rightMotor.enableVoltageCompensation(true);
    }

    // Return the shooter velocity in RPM
    public double getVelocity() {
        return integratedSensorUnitsToFlywheelRPM(leftMotor.getSelectedSensorVelocity());
    }

    // Sets the velocity of the shooter 
    public void setVelocity(double rpm) {
        lastSetpoint = rpm;
        if (rpm==0.0) {
            leftMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
            leftMotor.set(ControlMode.Velocity, flywheelRPMToIntegratedSensorUnits(rpm), DemandType.ArbitraryFeedForward, (rpm+261.0)/6163.0);
        }
    }

    public double getLastSetpoint() {
        return lastSetpoint;
    }

    public boolean atSetpoint() {
        boolean atSetpoint = BreadUtil.atReference(getVelocity(), getLastSetpoint(), 100, true);
        SmartDashboard.putBoolean("Shooter At Setpoint", atSetpoint);
        return atSetpoint;
    }
    
    private double integratedSensorUnitsToFlywheelRPM(double integratedSensorUnits) {
        return integratedSensorUnits * (600/2048.0);
    }

    private double flywheelRPMToIntegratedSensorUnits(double flywheelRPM) {
        return flywheelRPM * (2048.0/600);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
    }
    
}
