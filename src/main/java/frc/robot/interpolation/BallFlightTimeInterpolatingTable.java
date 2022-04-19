package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;
import static frc.robot.Constants.Vision.*;

public class BallFlightTimeInterpolatingTable {

    private BallFlightTimeInterpolatingTable() {}

    private static double scalar = 1.0; // TODO Change back
    // private static double scalar = 0.6067961165;

    private static TreeMap<Double, Double> table = new TreeMap<>(
        Map.ofEntries(
            entry(2.106467 + CAMERA_TO_CENTER, 1.232 * scalar),
            entry(2.425149 + CAMERA_TO_CENTER, 1.265 * scalar),
            entry(2.644736 + CAMERA_TO_CENTER, 1.237 * scalar),
            entry(2.930702 + CAMERA_TO_CENTER, 1.235 * scalar),
            entry(3.234261 + CAMERA_TO_CENTER, 1.28 * scalar),
            entry(3.535450 + CAMERA_TO_CENTER, 1.266 * scalar),
            entry(Double.MAX_VALUE, 1.266 * scalar)

        )
    );

    public static double get(double distanceToCenterOfHub) {
        Entry<Double, Double> ceil = table.ceilingEntry(distanceToCenterOfHub);
        Entry<Double, Double> floor = table.floorEntry(distanceToCenterOfHub);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return lerp(
            floor.getValue(),
            ceil.getValue(),
            (distanceToCenterOfHub - floor.getKey()) / (ceil.getKey() - floor.getKey())
        );
    }

    private static double lerp(double y1, double y2, double t) {
        return y1 + (y2 - y1) * t;
    }
    
}
