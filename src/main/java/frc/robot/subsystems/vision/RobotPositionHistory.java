package frc.robot.subsystems.vision;

import java.util.TreeMap;
import java.util.Map.Entry;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commons.TimestampedPose2d;

public class RobotPositionHistory {

    private static TreeMap<Double, Pose2d> timeInterpolatingTreeMap = new TreeMap<Double, Pose2d>();

    public static void update(double timestamp, Pose2d pose) {
        timeInterpolatingTreeMap.put(timestamp, pose);

        if (timeInterpolatingTreeMap.size() > 50.0) {
            timeInterpolatingTreeMap.pollFirstEntry();
        }
    }

    public static Pose2d get(double timestamp) {
        Entry<Double, Pose2d> ceil = timeInterpolatingTreeMap.ceilingEntry(timestamp);
        Entry<Double, Pose2d> floor = timeInterpolatingTreeMap.floorEntry(timestamp);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        double t = (timestamp - floor.getKey()) / (ceil.getKey() - floor.getKey()); // Value between 0 and 1 signifying the distance the interpolated point is between the ceiling and floor keys
        return new TimestampedPose2d(
            timestamp, 
            floor.getValue().interpolate(ceil.getValue(), t)
        );
    }

    public static void clear() {
        timeInterpolatingTreeMap.clear();
    }
    
}
