package frc.robot.drivers;

import com.revrobotics.CANSparkMax;

public class LazyCANSparkMax extends CANSparkMax {

    protected double lastSet = Double.NaN;
    protected Output lastControlMode = null;

    public LazyCANSparkMax(int id, MotorType motorType) {
        super(id, motorType);
    }

    @Override
    public void set(double speed) {
        if (lastControlMode != Output.Percent || speed != lastSet) {
            lastControlMode = Output.Percent;
            lastSet = speed;
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) {
        if (lastControlMode != Output.Voltage || outputVolts != lastSet) {
            lastControlMode = Output.Voltage;
            lastSet = outputVolts;
            super.set(outputVolts);
        }
    }

    public enum Output {
        Percent, Voltage
    }
    
}
