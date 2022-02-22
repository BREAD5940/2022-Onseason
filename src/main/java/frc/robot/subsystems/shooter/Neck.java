package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.BeamBreak;
import static frc.robot.Constants.Neck.*;

public class Neck extends SubsystemBase {

    private final TalonFX neck = new TalonFX(NECK_ID);
    private final BeamBreak topBeamBreak = new BeamBreak(TOP_BEAM_BREAK_CHANNEL);

    public Neck() {
        TalonFXConfiguration throatConfig = new TalonFXConfiguration();
        throatConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        throatConfig.slot0.kP = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0.02) * 1023.0; // TODO check this
        throatConfig.slot0.kI = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0) * 1023.0;
        throatConfig.slot0.kD = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0) * 1023.0;
        throatConfig.slot0.kF = 1023.0/neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(MAX_NECK_SURFACE_SPEED);
        throatConfig.slot0.closedLoopPeakOutput = 1.0;
        throatConfig.peakOutputForward = 1.0;
        throatConfig.peakOutputReverse = -1.0;
        throatConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
        neck.setNeutralMode(NeutralMode.Brake);
        neck.setInverted(TalonFXInvertType.Clockwise);
        neck.set(ControlMode.Velocity, 0.0);
        neck.configAllSettings(throatConfig);
        neck.selectProfileSlot(0, 0);
    }

    public boolean getTopBeamBreak() {
        return topBeamBreak.get();
    }

    public double getSurfaceSpeed() {
        return integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(neck.getSelectedSensorVelocity());
    }

    public void setSurfaceSpeed(double speedMetersPerSecond) {
        neck.set(ControlMode.Velocity, neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speedMetersPerSecond));
    }

    private double integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * (NECK_GEARING * (600.0/2048.0) * Math.PI * NECK_PULLEY_DIAMETER) / 60.0;
    }
    
    private double neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(double surfaceSpeed) {
        return surfaceSpeed * 60.0 / (NECK_GEARING * (600.0/2048.0) * Math.PI * NECK_PULLEY_DIAMETER);
    }
}
