package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Hood.*;

public class Hood extends SubsystemBase {

    public final CANSparkMax hood = new CANSparkMax(15, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hood.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    private final SparkMaxPIDController pid = hood.getPIDController();
    private double lastSetpoint = 0.0;

    public Hood() {
        hoodEncoder.setPositionConversionFactor(GEARING * 360.0);
        pid.setFeedbackDevice(hoodEncoder);
        pid.setP(0.5);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setOutputRange(-1, 1);
        hood.setSmartCurrentLimit(80);
        hood.setSecondaryCurrentLimit(70.0);
    }

    public void setPosition(double degrees) {
        double adjustedSetpoint = MathUtil.clamp(degrees, 1.0, 35.0);
        lastSetpoint = adjustedSetpoint;
        pid.setReference(adjustedSetpoint, CANSparkMax.ControlType.kPosition);
    }

    public double getSetpoint() {
        return lastSetpoint;
    }

    public double getPosition() {
        return hoodEncoder.getPosition();
    }

    public void reset() {
        hoodEncoder.setPosition(0.0);
    }

    public boolean atReference() {
        boolean atSetpoint = BreadUtil.atReference(getPosition(), getSetpoint(), 1, true);
        SmartDashboard.putBoolean("Hood at Setpoint", atSetpoint);
        return atSetpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", getPosition());
    }
    
}
