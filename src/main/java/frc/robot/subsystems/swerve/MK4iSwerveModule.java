package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

    public MK4iSwerveModule(int driveID, int steerID, int azimuthID, Rotation2d offset, boolean driveReversed, boolean steerReversed, boolean azimuthReversed) {

        // Configure the driving motor
        drive = new WPI_TalonFX(driveID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot0.kP = BreadUtil.integratedSensorUnitsToRadiansPerSecond(0.001) * WHEEL_RADIUS * 1023.0; // Check tomorrow
        driveConfig.slot0.kI = BreadUtil.integratedSensorUnitsToRadiansPerSecond(0.0) * WHEEL_RADIUS * 1023.0;
        driveConfig.slot0.kD = BreadUtil.integratedSensorUnitsToRadiansPerSecond(0.0) * WHEEL_RADIUS * 1023.0;
        driveConfig.slot0.kF = 1023.0/BreadUtil.radiansPerSecondToIntegratedSensorUnits(ROBOT_MAX_SPEED/WHEEL_RADIUS);
        driveConfig.slot0.closedLoopPeakOutput = 1.0;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        drive.setInverted(driveReversed);
        drive.setNeutralMode(NeutralMode.Brake);
        drive.configAllSettings(driveConfig);
        drive.set(ControlMode.Velocity, 0.0);
        drive.enableVoltageCompensation(true);
        drive.selectProfileSlot(0, 0);

        // Create the CAN Coder object
        azimuth = new CANCoder(azimuthID);
        CANCoderConfiguration azimuthConfig = new CANCoderConfiguration();
        azimuthConfig.magnetOffsetDegrees = offset.getDegrees();
        azimuthConfig.sensorDirection = false;
        azimuthConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        azimuthConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        azimuth.configAllSettings(azimuthConfig);

        // Configure the steering motor
        steer = new WPI_TalonFX(steerID);
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.remoteFilter0.remoteSensorDeviceID = azimuth.getDeviceID();
        steerConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        steerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        steerConfig.slot0.kP = BreadUtil.CANCoderSensorUnitsToRadians(1.0) * 1023.0;
        steerConfig.slot0.kI = BreadUtil.CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.kD = BreadUtil.CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.closedLoopPeakOutput = 1.0;
        steerConfig.peakOutputForward = 1.0;
        steerConfig.peakOutputReverse = -1.0;
        steer.setInverted(steerReversed);
        steer.setNeutralMode(NeutralMode.Brake);
        steer.setSensorPhase(true);
        steer.set(ControlMode.Velocity, 0.0);
        steer.configAllSettings(steerConfig);
        steer.selectProfileSlot(0, 0);

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

    public void setState(SwerveModuleState desiredState) {
        double[] state = getContinousOutput(
            SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()))
        );
        drive.set(ControlMode.Velocity, BreadUtil.radiansPerSecondToIntegratedSensorUnits(state[0]/WHEEL_RADIUS));
        steer.set(TalonFXControlMode.Position, BreadUtil.radiansToCANCoderSensorUnits(state[1]));
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

}

