package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/* climber subsystem hardware interface */
public interface ClimberIO {
    /* contains all of the input data recieved from the hardware */
    public static class ClimberIOInputs implements LoggableInputs {
        public boolean solenoidForward = false;
        public double posMeters = 0.0;
        public double velMetersPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double[] currentAmps = new double[] {}; // {leader, follower}
        public double[] tempCelcius = new double[] {}; // {leader, follower}

        public void toLog(LogTable table) {
            table.put("SolenoidForward", solenoidForward);
            table.put("PosMeters", posMeters);
            table.put("VelMetersPerSecond", velMetersPerSecond);
            table.put("AppliedVoltage", appliedVoltage);
            table.put("CurrentAmps", currentAmps);
            table.put("TempCelcius", tempCelcius);
        }

        public void fromLog(LogTable table) {
            solenoidForward = table.getBoolean("SolenoidForward", solenoidForward);
            posMeters = table.getDouble("PosMeters", posMeters);
            velMetersPerSecond = table.getDouble("VelMetersPerSecond", velMetersPerSecond);
            appliedVoltage = table.getDouble("AppliedVoltage", appliedVoltage);
            currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
            tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
        }
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ClimberIOInputs inputs) {}

    /* Sets the climber to a height setpoint via motion magic */
    public default void setHeight(double heightMeters, boolean fightingGravity) {}

    /* Sets the climber's neutral mode */
    public default void setNeutralMode(NeutralMode mode) {}

    /* Sets the pistons forwards or reversed */
    public default void setPistonsForward(boolean set) {}
}
