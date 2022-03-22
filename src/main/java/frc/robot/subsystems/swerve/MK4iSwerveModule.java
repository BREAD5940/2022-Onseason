// package frc.robot.subsystems.swerve;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.util.Units;
// import frc.robot.commons.BreadUtil;

// import static frc.robot.Constants.Drive.*;

// public class MK4iSwerveModule {

//     public final TalonFX steer;
//     public final TalonFX drive;
//     public final CANCoder azimuth;
//     private double kAngleReference;
//     private double kVelocityReference;

//     public MK4iSwerveModule(int driveID, int steerID, int azimuthID, Rotation2d offset, TalonFXInvertType driveDirection, boolean steerReversed, boolean azimuthReversed, String moduleIdentifier) {

//         // Configure the driving motor
//         drive = new TalonFX(driveID);
//         TalonFXConfiguration driveConfig = new TalonFXConfiguration();
//         driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
//         driveConfig.slot0.kP = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.001) * 1023.0; // TODO check this
//         driveConfig.slot0.kI = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
//         driveConfig.slot0.kD = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
//         driveConfig.slot0.kF = 1023.0/wheelSpeedMetersPerSecondToIntegratedSensorUnits(ROBOT_MAX_SPEED);
//         driveConfig.slot0.closedLoopPeakOutput = 1.0;
//         driveConfig.peakOutputForward = 1.0;
//         driveConfig.peakOutputReverse = -1.0;
//         driveConfig.voltageCompSaturation = 12.0;
//         driveConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80, 80, 1.5);
//         drive.setInverted(driveDirection);
//         drive.setNeutralMode(NeutralMode.Brake);
//         drive.configAllSettings(driveConfig);
//         drive.set(ControlMode.Velocity, 0.0);
//         drive.enableVoltageCompensation(true);
//         drive.selectProfileSlot(0, 0);
//         drive.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
//         drive.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

//         // Create CAN Coder object
//         azimuth = new CANCoder(azimuthID);
//         CANCoderConfiguration azimuthConfig = new CANCoderConfiguration();
//         azimuthConfig.magnetOffsetDegrees = offset.getDegrees();
//         azimuthConfig.sensorDirection = false;
//         azimuthConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//         azimuthConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360; // CANCoder will boot to a value between 0 and 360
//         azimuth.configAllSettings(azimuthConfig);

//         // Configure the steering motor
//         steer = new TalonFX(steerID);
//         TalonFXConfiguration steerConfig = new TalonFXConfiguration();
//         steerConfig.remoteFilter0.remoteSensorDeviceID = azimuth.getDeviceID();
//         steerConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
//         steerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
//         steerConfig.slot0.kP = CANCoderSensorUnitsToRadians(1.0) * 1023.0;
//         steerConfig.slot0.kI = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
//         steerConfig.slot0.kD = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
//         steerConfig.slot0.closedLoopPeakOutput = 1.0;
//         steerConfig.peakOutputForward = 1.0;
//         steerConfig.peakOutputReverse = -1.0;
//         // steerConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 30, 30, 1.5);
//         steer.setInverted(steerReversed);
//         steer.setNeutralMode(NeutralMode.Brake);
//         steer.setSensorPhase(true);
//         steer.set(ControlMode.Velocity, 0.0);
//         steer.configAllSettings(steerConfig);
//         steer.selectProfileSlot(0, 0);
//         steer.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
//         steer.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
//     }

//     // Returns the velocity in meters/second of the module
//     public double getVelocity() {
//         return (MODULE_GEARING * drive.getSelectedSensorVelocity() * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
//     }

//     // Returns the continous angle (in radians) of the swerve module
//     public double getAngle() {
//         return Units.degreesToRadians(azimuth.getPosition());
//     }
    
//     // Returns the absolute angle (in radians) of the swerve module
//     public double getAbsoluteAngle() {
//         return Units.degreesToRadians(azimuth.getAbsolutePosition());
//     }

//     // Returns the reference angle of the swerve module
//     public double getReferenceAngle() {
//         return kAngleReference;
//     }

//     // Returns the velocity of the swerve module
//     public double getReferenceVelocity() {
//         return kVelocityReference;
//     }

//     // Returns the state of the swerve module
//     public SwerveModuleState getState() {
//         return new SwerveModuleState(getVelocity(), new Rotation2d(getAngle()));
//     }

//     // Returns the percent output of the drive motor on the swerve module
//     public double getDriveOutputPercent() {
//         return drive.getMotorOutputPercent();
//     }

//     // Sets the desired State of the swerve module
//     public void setState(SwerveModuleState desiredState) {
//         // Deconstruct the swerve module state
//         double speedMetersPerSecond = desiredState.speedMetersPerSecond;
//         double angle = desiredState.angle.getRadians();
//         angle = BreadUtil.getRadians0to2PI(angle);

//         double difference = angle - getAbsoluteAngle();
//         // Change the target angle so the difference is in the range [-PI, PI]
//         if (difference >= Math.PI) {
//             angle -= 2.0 * Math.PI;
//         } else if (difference < -Math.PI) {
//             angle += 2.0 * Math.PI;
//         }
//         difference = angle - getAbsoluteAngle(); // Recalculate the difference

//         // If the difference is great than 90 degrees or less than -90 degrees
//         // The drive motor can be inverted so the total movement of the module is less than
//         // 90 degrees
//         if (difference > Math.PI/2.0 || difference < -Math.PI/2.0) {
//             angle += Math.PI;
//             speedMetersPerSecond *= -1;
//         }

//         // Put the target angle back into the range [0, 2PI]
//         angle = BreadUtil.getRadians0to2PI(angle);

//         // Apply outputs
//         setReferenceVelocity(speedMetersPerSecond);
//         setReferenceAngle(angle);

//         kVelocityReference = speedMetersPerSecond;
//     }

//     // Sets the reference velocity of the drive motor
//     private void setReferenceVelocity(double velocity) {
//         drive.set(ControlMode.Velocity, wheelSpeedMetersPerSecondToIntegratedSensorUnits(velocity));
//     }

//     // Sets the reference angle of the steer motor
//     private void setReferenceAngle(double referenceAngleRadians) {
//         double currentAngleRadians = getAngle();

//         double currentAngleRadiansMod = currentAngleRadians % (2 * Math.PI);
//         if (currentAngleRadians < 0.0) {
//             currentAngleRadians += 2 * Math.PI;
//         }

//         // The reference angle is between [0, 2 * PI] but the CANCoders can go above that
//         double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
//         if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
//             adjustedReferenceAngleRadians -= 2.0 * Math.PI;
//         } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
//             adjustedReferenceAngleRadians += 2.0 * Math.PI;
//         }

//         steer.set(TalonFXControlMode.Position, radiansToCANCoderSensorUnits(adjustedReferenceAngleRadians));

//         kAngleReference = adjustedReferenceAngleRadians;
//     }

//     private double CANCoderSensorUnitsToRadians(double sensorUnits) {
//         return sensorUnits * (2.0 * Math.PI)/CANCODER_RESOLUTION;
//     }

//     private double radiansToCANCoderSensorUnits(double radians) {
//         return radians * CANCODER_RESOLUTION/(2.0 * Math.PI);
//     }

//     private double integratedSensorUnitsToWheelSpeedMetersPerSecond(double integratedSensorUnits) {
//         return integratedSensorUnits * (MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
//     }

//     private double wheelSpeedMetersPerSecondToIntegratedSensorUnits(double wheelSpeed) {
//         return wheelSpeed * 60.0 / (MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS);
//     }  

// }

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Drive.*;

public class MK4iSwerveModule {

    public final TalonFX steer;
    public final TalonFX drive;
    public final CANCoder azimuth;
    private double[] kDesiredState = {0, 0};

    public MK4iSwerveModule(int driveID, int steerID, int azimuthID, Rotation2d offset, TalonFXInvertType driveDirection, boolean steerReversed, boolean azimuthReversed, String moduleIdentifier) {

        // Configure the driving motor
        drive = new TalonFX(driveID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot0.kP = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.001) * 1023.0; // TODO check this
        driveConfig.slot0.kI = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
        driveConfig.slot0.kD = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
        driveConfig.slot0.kF = 1023.0/wheelSpeedMetersPerSecondToIntegratedSensorUnits(ROBOT_MAX_SPEED);
        driveConfig.slot0.closedLoopPeakOutput = 1.0;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        driveConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        drive.setInverted(driveDirection);
        drive.setNeutralMode(NeutralMode.Brake);
        drive.configAllSettings(driveConfig);
        drive.set(ControlMode.Velocity, 0.0);
        drive.enableVoltageCompensation(true);
        drive.selectProfileSlot(0, 0);
        drive.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        drive.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        // Create CAN Coder object
        azimuth = new CANCoder(azimuthID);
        CANCoderConfiguration azimuthConfig = new CANCoderConfiguration();
        azimuthConfig.magnetOffsetDegrees = offset.getDegrees();
        azimuthConfig.sensorDirection = false;
        azimuthConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        azimuthConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        azimuth.configAllSettings(azimuthConfig);

        // Configure the steering motor
        steer = new TalonFX(steerID);
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.remoteFilter0.remoteSensorDeviceID = azimuth.getDeviceID();
        steerConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        steerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        steerConfig.slot0.kP = CANCoderSensorUnitsToRadians(1.0) * 1023.0;
        steerConfig.slot0.kI = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.kD = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.closedLoopPeakOutput = 1.0;
        steerConfig.peakOutputForward = 1.0;
        steerConfig.peakOutputReverse = -1.0;
        steerConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50.0, 50.0, 1.5);
        steer.setInverted(steerReversed);
        steer.setNeutralMode(NeutralMode.Brake);
        steer.setSensorPhase(true);
        steer.set(ControlMode.Velocity, 0.0);
        steer.configAllSettings(steerConfig);
        steer.selectProfileSlot(0, 0);
        steer.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        steer.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    }

    public double getVelocity() {
        return (MODULE_GEARING * drive.getSelectedSensorVelocity() * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
    }

    public double getAngle() {
        return Units.degreesToRadians(azimuth.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getAngle()));
    }

    public double[] getDesiredState() {
        return kDesiredState;
    }

    public double getMotorOutputPercent() {
        return drive.getMotorOutputPercent();
    }

    public void setState(SwerveModuleState desiredState) {
        double[] state = getContinousOutput(
            SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()))
        );
        kDesiredState = state;
        drive.set(ControlMode.Velocity, wheelSpeedMetersPerSecondToIntegratedSensorUnits(state[0]));
        steer.set(TalonFXControlMode.Position, radiansToCANCoderSensorUnits(state[1]));
    }

    private double[] getContinousOutput(SwerveModuleState desiredState) {
        double currentAngle = getAngle();
        double absoluteHeading = currentAngle % (2.0 * Math.PI);
        if (absoluteHeading < 0.0) {
            absoluteHeading += 2.0 * Math.PI;
        }
        double adjustedDesiredAngle = BreadUtil.getRadians0To2PI(desiredState.angle) + currentAngle - absoluteHeading;
        if (BreadUtil.getRadians0To2PI(desiredState.angle) - absoluteHeading > Math.PI) {
            return new double[] {
                desiredState.speedMetersPerSecond,
                adjustedDesiredAngle - 2.0 * Math.PI
            };
        } else if (BreadUtil.getRadians0To2PI(desiredState.angle) - absoluteHeading < -Math.PI) {
            return new double[] {
                desiredState.speedMetersPerSecond,
                adjustedDesiredAngle + 2.0 * Math.PI
            };
        } else {
            return new double[] {
                desiredState.speedMetersPerSecond,
                adjustedDesiredAngle
            };
        }
    }

    private final double CANCoderSensorUnitsToRadians(double sensorUnits) {
        return sensorUnits * (2.0 * Math.PI)/CANCODER_RESOLUTION;
    }

    private final double radiansToCANCoderSensorUnits(double radians) {
        return radians * CANCODER_RESOLUTION/(2.0 * Math.PI);
    }

    private final double integratedSensorUnitsToWheelSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * (MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
    }

    private final double wheelSpeedMetersPerSecondToIntegratedSensorUnits(double wheelSpeed) {
        return wheelSpeed * 60.0 / (MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS);
    }  

}


