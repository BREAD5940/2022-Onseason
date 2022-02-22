package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BeamBreak;
import frc.robot.sensors.ColorSensor;
import frc.robot.sensors.ColorSensor.BallColor;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import static frc.robot.Constants.Gut.*;

public class Gut extends SubsystemBase {
    
    private final TalonFX gut = new TalonFX(GUT_ID);
    private final BeamBreak leftBeamBreak = new BeamBreak(LEFT_BEAM_BREAK_CHANNEL);
    private final BeamBreak middleBeamBreak = new BeamBreak(MIDDLE_BEAM_BREAK_CHANNEL);
    private final BeamBreak rightBeamBreak = new BeamBreak(RIGHT_BEAM_BREAK_CHANNEL);
    private final ColorSensor colorSensor = new ColorSensor(
        COLOR_SENSOR_PORT,
        COLOR_SENSOR_RED_TARGET,
        COLOR_SENSOR_BLUE_TARGET,
        COLOR_SENSOR_NO_TARGET
    );
    private double lastSetpoint = 0.0;

    public Gut() {
        TalonFXConfiguration gutConfig = new TalonFXConfiguration();
        gutConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        gutConfig.slot0.kP = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0.1) * 1023.0; // TODO check this 
        gutConfig.slot0.kI = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0) * 1023.0;
        gutConfig.slot0.kD = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0) * 1023.0;
        gutConfig.slot0.kF = 1023.0/gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(MAX_GUT_SURFACE_SPEED);
        gutConfig.slot0.closedLoopPeakOutput = 1.0;
        gutConfig.peakOutputForward = 1.0;
        gutConfig.peakOutputReverse = -1.0;
        gutConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
        gutConfig.velocityMeasurementWindow = 8;
        gut.setNeutralMode(NeutralMode.Brake);
        gut.set(ControlMode.Velocity, 0.0);
        gut.setInverted(TalonFXInvertType.Clockwise);
        gut.configAllSettings(gutConfig);
        gut.selectProfileSlot(0, 0);
    }

    public boolean getLeftBeamBreak() {
        return leftBeamBreak.get();
    } 

    public boolean getMiddleBeamBreak() {
        return middleBeamBreak.get();
    }

    public boolean getRightBeamBreak() {
        return rightBeamBreak.get();
    }

    public BallColor getColorSensor() {
        return colorSensor.get();
    }

    public double getSurfaceSpeed() {
        return integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(gut.getSelectedSensorVelocity());
    }

    public double getLastSetpoint() {
        return lastSetpoint;
    }

    public void setSurfaceSpeed(double speedMetersPerSecond) {
        lastSetpoint = speedMetersPerSecond;
        gut.set(ControlMode.Velocity, gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speedMetersPerSecond));
    }

    private double integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((GUT_GEARING * (600.0/2048.0) * Math.PI * GUT_PULLEY_DIAMETER) / 60.0);
    }

    private double gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(double gutSurfaceSpeed) {
        return gutSurfaceSpeed * (60.0/(GUT_GEARING * (600.0/2048.0) * Math.PI * GUT_PULLEY_DIAMETER));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gut Setpoint", lastSetpoint);
        SmartDashboard.putNumber("Gut Velocity", getSurfaceSpeed());
        SmartDashboard.putBoolean("Middle Beam Break", getMiddleBeamBreak());
    }

}
