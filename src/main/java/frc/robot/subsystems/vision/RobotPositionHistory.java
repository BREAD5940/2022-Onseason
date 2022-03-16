package frc.robot.subsystems.vision;

import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotPositionHistory {

    private static TreeMap<Double, Rotation2d> timeInterpolatingTreeMap = new TreeMap<Double, Rotation2d>();

    public static void update(double timestamp, Rotation2d robotAngle) {
        timeInterpolatingTreeMap.put(timestamp, robotAngle);

        if (timeInterpolatingTreeMap.size() > 50.0) {
            timeInterpolatingTreeMap.pollFirstEntry();
        }
    }

    public static Rotation2d get(double timestamp) {
        Entry<Double, Rotation2d> ceil = timeInterpolatingTreeMap.ceilingEntry(timestamp);
        Entry<Double, Rotation2d> floor = timeInterpolatingTreeMap.floorEntry(timestamp);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        double t = (timestamp - floor.getKey()) / (ceil.getKey() - floor.getKey()); // Value between 0 and 1 signifying the distance the interpolated point is between the ceiling and floor keys
        return lerp(ceil.getValue(), floor.getValue(), t);
    }

    public static void clear() {
        timeInterpolatingTreeMap.clear();
    }

    private static Rotation2d lerp(Rotation2d y2, Rotation2d y1, double t) {
        return y1.plus((y2.minus(y1)).times(t));
        // y1 + (y2 - y1) * t
    }
    
}
