package frc.robot.subsystems.gutneck;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.sensors.ColorSensor.BallColor;

/* gutneck subsystem hardware interface */
public interface GutNeckIO {
    /* contains all of the input data recieved from the hardware */
    public static class GutNeckIOInputs implements LoggableInputs {
        public double gutVelocityMetersPerSecond = 0.0;
        public double gutAppliedVoltage = 0.0;
        public double gutCurrentAmps = 0.0;
        public double gutTempCelcius = 0.0;
        public double neckVelocityMetersPerSecond = 0.0;
        public double neckAppliedVoltage = 0.0;
        public double neckCurrentAmps = 0.0;
        public double neckTempCelcius = 0.0;
        public boolean bottomBeamBreakBroken = false;
        public boolean topBeamBreakBroken = false;
        public BallColor detectedColor = BallColor.NONE;

        @Override
        public void toLog(LogTable table) {
            table.put("GutVelocityMetersPerSecond", gutVelocityMetersPerSecond);
            table.put("GutAppliedVoltage", gutAppliedVoltage);
            table.put("GutCurrentAmps", gutCurrentAmps);
            table.put("GutTempCelcius", gutTempCelcius);
            table.put("NeckVelocityMetersPerSecond", neckVelocityMetersPerSecond);
            table.put("NeckAppliedVoltage", neckAppliedVoltage);
            table.put("NeckCurrentAmps", neckCurrentAmps);
            table.put("NeckTempCelcius", neckTempCelcius);
            table.put("BottomBeamBreakBroken", bottomBeamBreakBroken);
            table.put("TopBeamBreakBroken", topBeamBreakBroken);
            table.put("DetectedColor", detectedColor.toString());
        } 

        @Override
        public void fromLog(LogTable table) {
            gutVelocityMetersPerSecond = table.getDouble("GutVelocityMetersPerSecond", gutVelocityMetersPerSecond);
            gutAppliedVoltage = table.getDouble("GutAppliedVoltage", gutAppliedVoltage);
            gutCurrentAmps = table.getDouble("GutCurrentAmps", gutCurrentAmps);
            gutTempCelcius = table.getDouble("GutTempCelcius", gutTempCelcius);
            neckVelocityMetersPerSecond = table.getDouble("NeckVelocityMetersPerSecond", neckVelocityMetersPerSecond);
            neckAppliedVoltage = table.getDouble("NeckAppliedVoltage", neckAppliedVoltage);
            neckCurrentAmps = table.getDouble("NeckCurrentAmps", neckCurrentAmps);
            neckTempCelcius = table.getDouble("NeckTempCelcius", neckTempCelcius);
            bottomBeamBreakBroken = table.getBoolean("BottomBeamBreakBroken", bottomBeamBreakBroken);
            topBeamBreakBroken = table.getBoolean("TopBeamBreakBroken", topBeamBreakBroken);
            detectedColor = BallColor.valueOf(table.getString("DetectedColor", detectedColor.toString()));
        }
        
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(GutNeckIOInputs inputs) { }

    /* Sets the speed of the gut in meters per second */
    public default void setGutSpeedMetersPerSecond(double speed) { }
    
    /* Sets the speed of the neck in meters per second */
    public default void setNeckSpeedMetersPerSecond(double speed) { }
}
