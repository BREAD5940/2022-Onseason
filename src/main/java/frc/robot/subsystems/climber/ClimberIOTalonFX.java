package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.drivers.TalonUtil;
import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class ClimberIOTalonFX implements ClimberIO {

    private final DoubleSolenoid solenoids;
    
    private final TalonFX leader;
    private final TalonFX follower;

    public ClimberIOTalonFX() {
        solenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, CLIMBER_FORWARD_CHANNEL, CLIMBER_REVERSE_CHANNEL);
        leader = new TalonFX(TOP_CLIMBER_MOTOR_ID);
        follower = new TalonFX(BOTTOM_CLIMBER_MOTOR_ID);

        /* configurations for the leader motor */
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.slot0.kP = integratedSensorUnitsToMetersPerSecond(1.0) * 1023.0;
        leaderConfig.slot0.kI = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        leaderConfig.slot0.kD = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        leaderConfig.slot0.kF = 1023.0/metersPerSecondToIntegratedSensorUnits(MAX_CLIMBER_TRAVEL_SPEED);
        leaderConfig.motionCruiseVelocity = metersPerSecondToIntegratedSensorUnits(1.4);
        leaderConfig.motionAcceleration = metersPerSecondToIntegratedSensorUnits(5.0);
        leaderConfig.voltageCompSaturation = 10.5;
        leader.setInverted(TOP_CLIMBER_MOTOR_INVERT_TYPE);
        leader.enableVoltageCompensation(true);
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        TalonUtil.checkError(leader.configAllSettings(leaderConfig), "Top Climber Motor Configuration Failed");
        leader.setSelectedSensorPosition(0.0);

        /* configurations for the follower motor */
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        follower.setInverted(BOTTOM_CLIMBER_MOTOR_INVERT_TYPE);
        TalonUtil.checkError(follower.configAllSettings(followerConfig), "Bottom Climber Motor Configuration Failed");
        follower.follow(leader);
        follower.setStatusFramePeriod(StatusFrame.Status_1_General, 197);
        follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 193);
    }

    /* converts integrated sensor units to meters */
    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER)/2048.0);
    }

    /* converts meters to integrated sensor units */
    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(CLIMBER_GEARING * Math.PI * CLIMBER_PITCH_DIAMETER));
    }

    /* converts integrated sensor units to meters per second */
    private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * (600.0/2048.0) * Math.PI * CLIMBER_PITCH_DIAMETER)/60.0);
    }

    /* converts meters per second to integrated sensor units */
    private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
        return metersPerSecond * (60.0/(CLIMBER_GEARING * (600.0/2048.0) * Math.PI * CLIMBER_PITCH_DIAMETER));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.solenoidForward = solenoids.get() == Value.kForward; // TODO check that this works
        inputs.posMeters = integratedSensorUnitsToMeters(leader.getSelectedSensorPosition());
        inputs.velMetersPerSecond = integratedSensorUnitsToMetersPerSecond(leader.getSelectedSensorVelocity());
        inputs.appliedVoltage = leader.getMotorOutputVoltage(); // TODO check that this works
        inputs.currentAmps = new double[] {leader.getSupplyCurrent(), follower.getSupplyCurrent()};
        inputs.tempCelcius = new double[] {leader.getTemperature(), follower.getTemperature()};
    }

    @Override
    public void setHeight(double heightMeters, boolean fightingGravity) {
        double output = metersToIntegratedSensorUnits(MathUtil.clamp(heightMeters, CLIMBER_MINIMUM_TRAVEL + 0.001, CLIMBER_MAXIMUM_TRAVEL - 0.001));
        leader.set(ControlMode.MotionMagic, output, DemandType.ArbitraryFeedForward, fightingGravity ? -0.16893148154 : 0.0);
    }

    @Override
    public void setNeutralMode(NeutralMode mode) {
        leader.setNeutralMode(mode);
        follower.setNeutralMode(mode);
    }

    @Override
    public void setPistonsForward(boolean set) {
        if ((solenoids.get() == Value.kForward || solenoids.get() == Value.kOff) && !set) {
            solenoids.set(Value.kReverse);
        } else if ((solenoids.get() == Value.kReverse || solenoids.get() == Value.kOff) && set) {
            solenoids.set(Value.kForward);
        }
    }
    
}
