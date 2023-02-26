package frc.robot.subsystems.swerve;

import java.util.TreeMap;
import java.util.Map.Entry;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotPositionHistory {

    private static TreeMap<Double, Pose2d> timeInterpolatingTreeMap = new TreeMap<Double, Pose2d>();

    public static void update(double timestamp, Pose2d pose) {
        timeInterpolatingTreeMap.put(timestamp, pose);

        if (timeInterpolatingTreeMap.size() > 50.0 * 20) {
            timeInterpolatingTreeMap.pollFirstEntry();
        }
    }

    public static Pose2d get(double timestamp) {
        Entry<Double, Pose2d> ceil = timeInterpolatingTreeMap.ceilingEntry(timestamp);
        Entry<Double, Pose2d> floor = timeInterpolatingTreeMap.floorEntry(timestamp);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return floor.getValue().interpolate(
            ceil.getValue(), 
            (timestamp - floor.getKey()) / (ceil.getKey() - floor.getKey())
        );
    }

    public static void clear() {
        timeInterpolatingTreeMap.clear();
    }
}