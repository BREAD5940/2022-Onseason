package frc.robot.subsystems.statemachines;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadLogger;
import frc.robot.commons.BreadUtil;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.drivers.TalonUtil;

import static frc.robot.Constants.Flywheel.*;
import static frc.robot.Constants.Hood.*;

import java.io.IOException;

public class Shooter extends SubsystemBase {

    // Flywheel Hardware 
    private final TalonFX leftFlywheelMotor = TalonFXFactory.createDefaultTalon(LEFT_MOTOR_ID);
    private final TalonFX rightFlywheelMotor = TalonFXFactory.createDefaultTalon(RIGHT_MOTOR_ID);

    // Hood hardware
    private final CANSparkMax hoodMotor = new CANSparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final SparkMaxPIDController hoodPID = hoodMotor.getPIDController();
    private final SparkMaxAnalogSensor hoodLimit = hoodMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

    // State logic
    private ShooterState systemState = ShooterState.HOMING;
    private double flywheelCalibration = FLYWHEEL_CALIBRATION;

    // Logging Code
    private BreadLogger flywheelLogger = new BreadLogger("FlywheelData");

    // State variables
    Timer homingTimer = new Timer();
    Timer stabalizingTimer = new Timer();
    private boolean requestHome = false;
    private boolean requestShoot = false;
    private double hoodSetpoint = 0.0;
    private double flywheelSetpoint = 0.0;

    public Shooter() {
        // Configure left flywheel motor
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

        // Configure right flywheel motor
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

        // Configure hood motor 
        hoodEncoder.setPositionConversionFactor(HOOD_GEARING * 360.0);
        hoodPID.setFeedbackDevice(hoodEncoder);
        hoodPID.setP(0.3);
        hoodPID.setI(0.0);
        hoodPID.setD(0.05);
        hoodPID.setOutputRange(-0.4, 0.4);
    }

    // Private method to set the hood setpoint
    private void commandHoodPosition(double degrees) {
        double adjustedSetpoint = MathUtil.clamp(degrees, MIN_HOOD_TRAVEL + 1, MAX_HOOD_TRAVEL - 1);
        if ((getHoodLimitSwitchTriggered() && degrees < getHoodPosition())) {
            hoodMotor.set(0.0);
        } else {
            hoodPID.setReference(adjustedSetpoint, CANSparkMax.ControlType.kPosition);
        }
    }

    // Private method to set the hood percent
    private void commandHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    // Private method to set the flywheel setpoint
    private void commandFlywheelVelocity(double rpm) {
        if (rpm==0.0) {
            leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
            leftFlywheelMotor.set(ControlMode.Velocity, flywheelRPMToIntegratedSensorUnits(rpm), DemandType.ArbitraryFeedForward, FeedForwardInterpolatingTable.get((rpm-50.0)/FLYWHEEL_GEARING));
            SmartDashboard.putNumber("Flywheel FF", FeedForwardInterpolatingTable.get((rpm-50.0)/FLYWHEEL_GEARING));
        }
    }

    // Private method to set the hood current limits
    private void setHoodCurrentLimits(int smart, double secondary) {
        hoodMotor.setSmartCurrentLimit(smart);
        hoodMotor.setSecondaryCurrentLimit(secondary);
    }

    // Converts integrated sensor units to flyweel rpm
    private double integratedSensorUnitsToFlywheelRPM(double integratedSensorUnits) {
        return integratedSensorUnits * ((600/2048.0) * FLYWHEEL_GEARING);
    }

    // Converts flywheel rpm to integrated sensor units
    private double flywheelRPMToIntegratedSensorUnits(double flywheelRPM) {
        return flywheelRPM / ((600/2048.0) * FLYWHEEL_GEARING);
    }

    // Requests the shooter to home
    public void requestHome() {
        requestHome = true;
    }

    // Requests the shooter rpm and hood angle
    public void requestShoot(double flywheelRPM, double hoodAngle) {
        if (flywheelRPM == 0.0) {
            requestIdle(); // Request idle because the flywheel rpm is 0.0
        } else {
            requestShoot = true;
            this.flywheelSetpoint = flywheelRPM * flywheelCalibration;
            this.hoodSetpoint = hoodAngle + 0.5;
        }

    }

    // Requests the shooter to go into idle
    public void requestIdle() {
        requestShoot = false;
    }

    // Returns the current state of the shooter
    public ShooterState getSystemState() {
        return systemState;
    }

    // Returns the hood position
    public double getHoodPosition() {
        return hoodEncoder.getPosition();
    }

    // Private method to reset the hood encoder
    public void resetHood(double newPosition) {
        hoodEncoder.setPosition(newPosition);
    }

    // Returns the hood velocity
    public double getHoodVelocity() {
        return hoodEncoder.getVelocity();
    }

    // Returns the voltage of the hood limit switch
    public boolean getHoodLimitSwitchTriggered() {
        return hoodLimit.getVoltage() < 1.5;
    }

    // Returns the flywheel velocity
    public double getFlywheelVelocity() {
        return integratedSensorUnitsToFlywheelRPM(leftFlywheelMotor.getSelectedSensorVelocity());
    }

    // Returns the hood setpoint
    public double getHoodSetpoint() {
        return hoodSetpoint;
    }

    // Returns the flywheel setpoint
    public double getFlywheelSetpoint() {
        return flywheelSetpoint;
    }

    // Returns whether the hood is at its setpoint
    public boolean hoodAtSetpoint() {
        return BreadUtil.atReference(getHoodPosition(), getHoodSetpoint(), 0.5, true);
    }

    // Returns whether the flywheel is at its setpoint
    public boolean flywheelAtSetpoint() {
        return BreadUtil.atReference(getFlywheelVelocity(), getFlywheelSetpoint(), 50.0, true);
    }

    // Shooter States
    public enum ShooterState {
        HOMING,
        IDLE, 
        APPROACHING_SETPOINT, 
        STABALIZING,
        AT_SETPOINT
    }
    
    // Handle statemachine in periodic
    @Override
    public void periodic() {
        ShooterState nextSystemState = systemState;
        if (systemState == ShooterState.HOMING) {
            // Outputs
            commandHoodVoltage(-1.5);
            commandFlywheelVelocity(0.0);

            // State transitions
            if (getHoodLimitSwitchTriggered()) {
                exitHomingSequence();
                nextSystemState = ShooterState.IDLE;
            }
        } else if (systemState == ShooterState.IDLE) {
            // Outputs
            commandHoodPosition(HOOD_IDLE_POS);
            commandFlywheelVelocity(SHOOTER_IDLE_VEL);

            // State transitions
            if (requestHome) {
                beginHomingSequence();
                nextSystemState = ShooterState.HOMING;
            } else if (requestShoot) {
                nextSystemState = ShooterState.APPROACHING_SETPOINT;
            } 
        } else if (systemState == ShooterState.APPROACHING_SETPOINT) {
            // Outputs
            commandHoodPosition(hoodSetpoint);
            commandFlywheelVelocity(flywheelSetpoint);

            try {
                flywheelLogger.write(RobotController.getFPGATime(), getFlywheelVelocity(), getFlywheelSetpoint());
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            // State transitions 
            if (requestHome) {
                beginHomingSequence();
                nextSystemState = ShooterState.HOMING;
            } else if (!requestShoot) {
                nextSystemState = ShooterState.IDLE;
            } else if (flywheelAtSetpoint()&&hoodAtSetpoint()) {
                beginStabalizing();
                nextSystemState = ShooterState.STABALIZING;
            } 
        } else if (systemState == ShooterState.STABALIZING) {
            // Outputs
            commandHoodPosition(hoodSetpoint);
            commandFlywheelVelocity(flywheelSetpoint);

            try {
                flywheelLogger.write(RobotController.getFPGATime(), getFlywheelVelocity(), getFlywheelSetpoint());
            } catch (IOException e) {
                e.printStackTrace();
            }

            // State transitions
            if (requestHome) {
                beginHomingSequence();
                nextSystemState = ShooterState.HOMING;
            } else if (!requestShoot) {
                nextSystemState = ShooterState.IDLE;
            } else if (stabalizingTimer.get() >= 0.25) {
                exitStabalizing();
                nextSystemState = ShooterState.AT_SETPOINT;
            } 
        } else if (systemState == ShooterState.AT_SETPOINT) {
            // Outputs
            commandHoodPosition(hoodSetpoint);
            commandFlywheelVelocity(flywheelSetpoint);

            try {
                flywheelLogger.write(RobotController.getFPGATime(), getFlywheelVelocity(), getFlywheelSetpoint());
            } catch (IOException e) {
                e.printStackTrace();
            }

            // State transitions
            if (requestHome) {
                nextSystemState = ShooterState.HOMING;
            } else if (!requestShoot) {
                nextSystemState = ShooterState.IDLE;
            } else if (!flywheelAtSetpoint()||!hoodAtSetpoint()) {
                nextSystemState = ShooterState.APPROACHING_SETPOINT;
            }
        }
        systemState = nextSystemState;
        SmartDashboard.putString("Shooter State", systemState.name());
        SmartDashboard.putNumber("Flywheel Velocity", getFlywheelVelocity());
        SmartDashboard.putNumber("Flywheel Setpoint", flywheelSetpoint);
        SmartDashboard.putNumber("Hood Angle", getHoodPosition());
        SmartDashboard.putNumber("Flywheel Motor Output", leftFlywheelMotor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Hood AtSetpoint", hoodAtSetpoint());
        SmartDashboard.putBoolean("FlywheelAtSetpoint", flywheelAtSetpoint());
        SmartDashboard.putBoolean("Hood Limit Switch Triggered", getHoodLimitSwitchTriggered());
        flywheelCalibration = SmartDashboard.getNumber("Flywheel Calibration", FLYWHEEL_CALIBRATION);
    }

    // Method to be called when you begin homing
    private void beginHomingSequence() {
        homingTimer.reset();
        homingTimer.start();
        setHoodCurrentLimits(5, 10.0);
    }   

    // Method to be called when you exit homing 
    private void exitHomingSequence() {
        homingTimer.stop();
        homingTimer.reset();
        setHoodCurrentLimits(30, 40.0);
        resetHood(0.0);
        commandHoodVoltage(0.0);
        requestHome = false;
    }

    // Method to be called when you begin stabalizing
    private void beginStabalizing() {
        stabalizingTimer.reset();
        stabalizingTimer.start();
    }

    // Method to be called when you exit stabalizing
    private void exitStabalizing() {
        stabalizingTimer.stop();
    }

     
}
