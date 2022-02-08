package frc.robot.commons;

import static frc.robot.Constants.Drive.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class BreadUtil {

    // Private constructor so that the class cannot be instantiated
    private BreadUtil() {}
    
    // Converts the CANCoder's raw sensor units to radians
    public static double CANCoderSensorUnitsToRadians(double CANCoderSensorUnits) {
        return CANCoderSensorUnits * ((2.0 * Math.PI)/CANCODER_RESOLUTION);
    }

    // Converts radians to the CANCoder's raw sensor units (for PIDF control) 
    public static double radiansToCANCoderSensorUnits(double radians) {
        return radians * (CANCODER_RESOLUTION/(2.0 * Math.PI));
    }
    
    // Converts the falcon 500's integrated velocity sensor's units to radians per second
    public static double integratedSensorUnitsToRadiansPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI) / 60.0);
    }

    // Converts radians per second to the falcon 500's integrated velocity sensor units
    public static double radiansPerSecondToIntegratedSensorUnits(double radiansPerSecond) {
        return radiansPerSecond * (60.0/(MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI));
    }

    // Converts a Rotation2d object to an double within the range of [0, 2pi]
    public static double getRadians0To2PI(Rotation2d angle) {
        return angle.getRadians() < 0.0 ? 2 * Math.PI + angle.getRadians() : angle.getRadians();
    }

}
