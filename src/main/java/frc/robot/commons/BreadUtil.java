package frc.robot.commons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BreadUtil {

    // Private constructor so that the class cannot be instantiated
    private BreadUtil() {}

    // Returns the angle to the target
    public static Rotation2d getAngleToTarget(Translation2d fieldToRobot, Translation2d fieldToTarget) {
        Translation2d targetToRobot = fieldToTarget.minus(fieldToRobot);
        return new Rotation2d(targetToRobot.getX(), targetToRobot.getY());
    }

    // Converts a Rotation2d object to a double within the range of [0, 2pi]
    public static double getRadians0To2PI(Rotation2d angle) {
        return angle.getRadians() < 0.0 ? 2 * Math.PI + angle.getRadians() : angle.getRadians();
    }

    // Converts an angle in radians to a double within the range of [0, 2pi]
    public static double getRadians0to2PI(double angle) {
        angle %= (2 * Math.PI);
        if (angle < 0.0) {
            angle += (2 * Math.PI);
        }
        return angle;
    }

    // At reference method
    public static boolean atReference(double val, double reference, double tolerance, boolean inclusive) {
        return inclusive ? (Math.abs(reference - val) <= tolerance) : (Math.abs(reference - val) < tolerance); 
    }

}
