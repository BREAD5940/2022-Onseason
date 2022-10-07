package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/* shooter subsystem hardware interface */
public interface ShooterIO {
    /* contains all of the input data recieved from the hardware */
    public static class ShooterIOInputs implements LoggableInputs {
        public double flywheelVelocityRotationsPerSecond = 0.0;
        public double flywheelAppliedVoltage = 0.0;
        public double[] flywheelCurrentAmps = new double[] {};
        public double[] flywheelTempCelcius = new double[] {};
        public double hoodPosDeg = 0.0;
        public double hoodAppliedVoltage = 0.0;
        public double hoodCurrentAmps = 0.0;
        public double hoodTempCelcius = 0.0;
        public boolean hoodLimitTriggered = false;

        @Override
        public void toLog(LogTable table) {
            table.put("FlywheelVelocityMetersPerSecond", flywheelVelocityRotationsPerSecond);
            table.put("FlywheelAppliedVoltage", flywheelAppliedVoltage);
            table.put("FlywheelCurrentAmps", flywheelCurrentAmps);
            table.put("FlywheelTempCelcius", flywheelTempCelcius);
            table.put("HoodPosDeg", hoodPosDeg);
            table.put("HoodAppliedVoltage", hoodAppliedVoltage);
            table.put("HoodCurrentAmps", hoodCurrentAmps);
            table.put("HoodTempCelcius", hoodTempCelcius);
            table.put("HoodLimitTriggered", hoodLimitTriggered);
        }

        @Override
        public void fromLog(LogTable table) {
            flywheelVelocityRotationsPerSecond = table.getDouble("FlywheelVelocityRotationsPerSecond", flywheelVelocityRotationsPerSecond);
            flywheelAppliedVoltage = table.getDouble("FlywheelAppliedVoltage", flywheelAppliedVoltage);
            flywheelCurrentAmps = table.getDoubleArray("FlywheelCurrentAmps", flywheelCurrentAmps);
            flywheelTempCelcius = table.getDoubleArray("FlywheelTempCelcius", flywheelTempCelcius);
            hoodPosDeg = table.getDouble("HoodPosDeg", hoodPosDeg);
            hoodAppliedVoltage = table.getDouble("HoodAppliedVoltage", hoodAppliedVoltage);
            hoodCurrentAmps = table.getDouble("HoodCurrentAmps", hoodCurrentAmps);
            hoodTempCelcius = table.getDouble("HoodTempCelcius", hoodTempCelcius);
            hoodLimitTriggered = table.getBoolean("HoodLimitTriggered", hoodLimitTriggered);
        }
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ShooterIOInputs inputs) { }

    /* Sets the current limits of the hood in amps */
    public default void setHoodCurrentLimits(int smart, double secondary) { }

    /* Resets the hood with the given position in degrees */
    public default void resetHood(double newPos) { }

    /* Sets the position of the hood in degrees */
    public default void setHoodPos(double posDeg) { }

    /* Sets the velocity of the flywheel in rotations per second */
    public default void setFlywheelVel(double velRPM) { }
}
