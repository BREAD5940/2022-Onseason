package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.drivers.TalonUtil;

import static frc.robot.Constants.Flywheel.*;
import static frc.robot.Constants.Hood.*;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    private final CANSparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkMaxPIDController hoodPID;
    private final SparkMaxAnalogSensor hoodLimit;

    // State variables
    public ShooterIOTalonFX() {
        leftFlywheelMotor = TalonFXFactory.createDefaultTalon(LEFT_MOTOR_ID);
        rightFlywheelMotor = TalonFXFactory.createDefaultTalon(RIGHT_MOTOR_ID);
        hoodMotor = new CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        hoodEncoder = hoodMotor.getEncoder();
        hoodPID = hoodMotor.getPIDController();
        hoodLimit = hoodMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

        /* configurations for the left flywheel motor */
        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.slot0.kP = integratedSensorUnitsToFlywheelRPM(0.0015) * 1023.0;
        leftMotorConfig.slot0.kI = integratedSensorUnitsToFlywheelRPM(0.0) * 1023.0;
        leftMotorConfig.slot0.kD = integratedSensorUnitsToFlywheelRPM(0.0) * 1023.0;
        leftMotorConfig.slot0.kF = 0.0;
        leftMotorConfig.slot0.closedLoopPeakOutput = 1.0;
        leftMotorConfig.peakOutputForward = 1.0;
        leftMotorConfig.peakOutputReverse = 0.0;
        leftMotorConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;
        leftMotorConfig.velocityMeasurementWindow = 8;
        leftMotorConfig.voltageCompSaturation = 9.5;
        leftMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80, 80, 1.5);
        TalonUtil.checkError(leftFlywheelMotor.configAllSettings(leftMotorConfig), "Left Flywheel Motor Configuration Failed");
        leftFlywheelMotor.selectProfileSlot(0, 0);
        leftFlywheelMotor.setInverted(LEFT_MOTOR_DIRECTION);
        leftFlywheelMotor.enableVoltageCompensation(true);
        leftFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        leftFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

        /* configurations for the right flywheel motor */
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.peakOutputForward = 1.0;
        rightMotorConfig.peakOutputReverse = -0.0;
        rightMotorConfig.voltageCompSaturation = 9.5;
        rightMotorConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 80, 80, 1.5);
        TalonUtil.checkError(rightFlywheelMotor.configAllSettings(rightMotorConfig), "Right Flywheel Motor Configuration Failed");
        rightFlywheelMotor.setInverted(RIGHT_MOTOR_DIRECTION);
        rightFlywheelMotor.follow(leftFlywheelMotor);
        rightFlywheelMotor.enableVoltageCompensation(true);
        rightFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 239);
        rightFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 233);

        /* configurations for the hood motor */
        hoodEncoder.setPositionConversionFactor(HOOD_GEARING * 360.0);
        hoodPID.setFeedbackDevice(hoodEncoder);
        hoodPID.setP(0.3);
        hoodPID.setI(0.0);
        hoodPID.setD(0.05);
        hoodPID.setOutputRange(-0.4, 0.4);
    }

    /* converts integrated sensor units to flywheel RPM */
    private double integratedSensorUnitsToFlywheelRPM(double integratedSensorUnits) {
        return integratedSensorUnits * ((600/2048.0) * FLYWHEEL_GEARING);
    }

    /* converts flywheel RPM to integrated sensor units */
    private double flywheelRPMToIntegratedSensorUnits(double flywheelRPM) {
        return flywheelRPM / ((600/2048.0) * FLYWHEEL_GEARING);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelVelocityRotationsPerSecond = integratedSensorUnitsToFlywheelRPM(leftFlywheelMotor.getSelectedSensorVelocity());
        inputs.flywheelAppliedVoltage = leftFlywheelMotor.getMotorOutputVoltage();
        inputs.flywheelCurrentAmps = new double[] {leftFlywheelMotor.getSupplyCurrent(), rightFlywheelMotor.getSupplyCurrent()};
        inputs.flywheelTempCelcius = new double[] {leftFlywheelMotor.getTemperature(), rightFlywheelMotor.getTemperature()};
        inputs.hoodPosDeg = hoodEncoder.getPosition();
        inputs.hoodAppliedVoltage = hoodMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.hoodCurrentAmps = hoodMotor.getOutputCurrent();
        inputs.hoodTempCelcius = hoodMotor.getMotorTemperature();
        inputs.hoodLimitTriggered = hoodLimit.getVoltage() < 1.5;
    }

    @Override
    public void setHoodCurrentLimits(int smart, double secondary) {
        hoodMotor.setSmartCurrentLimit(smart);
        hoodMotor.setSecondaryCurrentLimit(secondary);
    }

    @Override
    public void resetHood(double newPos) {
        hoodEncoder.setPosition(newPos);
    }

    @Override
    public void setHoodPos(double posDeg) {
        double adjustedSetpoint = MathUtil.clamp(posDeg, MIN_HOOD_TRAVEL + 1, MAX_HOOD_TRAVEL - 1); // TODO check that this works
        if ((hoodLimit.getVoltage() < 1.5) && posDeg < hoodEncoder.getPosition()) {
            hoodMotor.set(0.0);
        } else {
            hoodPID.setReference(adjustedSetpoint, CANSparkMax.ControlType.kPosition);
        }
    }

    @Override
    public void setFlywheelVel(double velRPM) {
        if (velRPM==0.0) {
            leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
            leftFlywheelMotor.set(ControlMode.Velocity, flywheelRPMToIntegratedSensorUnits(velRPM), DemandType.ArbitraryFeedForward, FeedForwardInterpolatingTable.get((velRPM-50.0)/FLYWHEEL_GEARING));
        }
    }

}
