package frc.robot.subsystems.gutneck;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.drivers.TalonFXFactory;
import frc.robot.drivers.TalonUtil;
import frc.robot.sensors.BeamBreak;
import frc.robot.sensors.ColorSensor;

import static frc.robot.Constants.Gut.*;
import static frc.robot.Constants.Neck.*;

public class GutNeckIOTalonFX implements GutNeckIO {

    /* Motors */
    private final TalonFX gutMotor = TalonFXFactory.createDefaultTalon(GUT_ID);
    private final TalonFX neckMotor = TalonFXFactory.createDefaultTalon(NECK_ID);

    /* Sensors */
    private final BeamBreak leftBeamBreak = new BeamBreak(LEFT_BEAM_BREAK_CHANNEL);
    private final BeamBreak rightBeamBreak = new BeamBreak(RIGHT_BEAM_BREAK_CHANNEL);
    private final BeamBreak bottomBeamBreak = new BeamBreak(MIDDLE_BEAM_BREAK_CHANNEL);
    private final BeamBreak topBeamBreak = new BeamBreak(TOP_BEAM_BREAK_CHANNEL);
    public final ColorSensor colorSensor = new ColorSensor();

    public GutNeckIOTalonFX() {
        /* Configure the gut motor */
        TalonFXConfiguration gutConfig = new TalonFXConfiguration();
        gutConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        gutConfig.slot0.kP = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0.1) * 1023.0; 
        gutConfig.slot0.kI = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0) * 1023.0;
        gutConfig.slot0.kD = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(0) * 1023.0;
        gutConfig.slot0.kF = 1023.0/gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(MAX_GUT_SURFACE_SPEED);
        gutConfig.velocityMeasurementWindow = 8;
        gutMotor.setNeutralMode(NeutralMode.Brake);
        gutMotor.set(ControlMode.Velocity, 0.0);
        gutMotor.setInverted(TalonFXInvertType.Clockwise);
        TalonUtil.checkError(gutMotor.configAllSettings(gutConfig), "Gut Motor Configuration Failed");
        gutMotor.selectProfileSlot(0, 0);
        gutMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 199);
        gutMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 197);

        /* Configure the neck motor */
        TalonFXConfiguration neckConfig = new TalonFXConfiguration();
        neckConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        neckConfig.slot0.kP = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0.02) * 1023.0; // TODO check this
        neckConfig.slot0.kI = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0) * 1023.0;
        neckConfig.slot0.kD = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(0) * 1023.0;
        neckConfig.slot0.kF = 1023.0/neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(MAX_NECK_SURFACE_SPEED);
        neckMotor.setNeutralMode(NeutralMode.Brake);
        neckMotor.setInverted(TalonFXInvertType.Clockwise);
        neckMotor.set(ControlMode.Velocity, 0.0);
        TalonUtil.checkError(neckMotor.configAllSettings(neckConfig), "Neck Motor Configuration Failed");
        neckMotor.selectProfileSlot(0, 0);
        neckMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 223);
        neckMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 211);
    }

    /* Converts integrated sensor units to the neck's surface speed */
    private double integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * (NECK_GEARING * (600.0/2048.0) * Math.PI * NECK_PULLEY_DIAMETER) / 60.0;
    }
    
    /* Converts the gut's surface speed to integrated sensor units */
    private double neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(double surfaceSpeed) {
        return surfaceSpeed * 60.0 / (NECK_GEARING * (600.0/2048.0) * Math.PI * NECK_PULLEY_DIAMETER);
    }

    /* Converts integrated sensor units to the gut's surface speed */
    private double integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((GUT_GEARING * (600.0/2048.0) * Math.PI * GUT_PULLEY_DIAMETER) / 60.0);
    }

    /* Converts the gut's surface speed to integrated sensor units */
    private double gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(double gutSurfaceSpeed) {
        return gutSurfaceSpeed * (60.0/(GUT_GEARING * (600.0/2048.0) * Math.PI * GUT_PULLEY_DIAMETER));
    }

    @Override
    public void updateInputs(GutNeckIOInputs inputs) {
        inputs.gutVelocityMetersPerSecond = integratedSensorUnitsToGutSurfaceSpeedMetersPerSecond(gutMotor.getSelectedSensorVelocity());
        inputs.gutAppliedVoltage = gutMotor.getMotorOutputVoltage();
        inputs.gutCurrentAmps = gutMotor.getSupplyCurrent();
        inputs.gutTempCelcius = gutMotor.getTemperature();
        inputs.neckVelocityMetersPerSecond = integratedSensorUnitsToNeckSurfaceSpeedMetersPerSecond(neckMotor.getSelectedSensorVelocity());
        inputs.neckAppliedVoltage = neckMotor.getMotorOutputVoltage();
        inputs.neckCurrentAmps = neckMotor.getSupplyCurrent();
        inputs.gutTempCelcius = neckMotor.getTemperature();
        inputs.bottomBeamBreakBroken = bottomBeamBreak.get();
        inputs.topBeamBreakBroken = topBeamBreak.get();
        inputs.detectedColor = colorSensor.get();
    }

    @Override
    public void setGutSpeedMetersPerSecond(double speed) {
        gutMotor.set(ControlMode.Velocity, gutSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speed));
    }

    @Override
    public void setNeckSpeedMetersPerSecond(double speed) {
        neckMotor.set(ControlMode.Velocity, neckSurfaceSpeedMetersPerSecondToIntegratedSensorUnits(speed));
    }
    
}
