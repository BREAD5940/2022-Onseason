package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import static java.util.Map.entry;

public class BallFlightTimeInterpolatingTable {

    private BallFlightTimeInterpolatingTable() {}

    private static TreeMap<Double, Double> table = new TreeMap<>(
        Map.ofEntries(
            entry(0.0, 1.36),
            entry(Double.MAX_VALUE, 1.36)
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
