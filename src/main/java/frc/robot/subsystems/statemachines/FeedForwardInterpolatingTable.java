package frc.robot.subsystems.statemachines;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

public class FeedForwardInterpolatingTable {

    private FeedForwardInterpolatingTable() {}

    public static TreeMap<Double, Double> table = new TreeMap<>(
        Map.ofEntries(
            entry(225.0, 0.1),
            entry(710.0, 0.2),
            entry(935.0, 0.25),
            entry(1157.0, 0.3),
            entry(1403.0, 0.35),
            entry(1643.0, 0.4),
            entry(2105.0, 0.5),
            entry(2350.0, 0.55),
            entry(2585.0, 0.6),
            entry(2843.0, 0.65),
            entry(3145.0, 0.7),
            entry(3388.0, 0.75),
            entry(3645.0, 0.8),
            entry(3900.0, 0.85),
            entry(4167.0, 0.9)
        )
    );

    public static double get(double flywheelRPM) {
        Entry<Double, Double> ceil = table.ceilingEntry(flywheelRPM);
        Entry<Double, Double> floor = table.floorEntry(flywheelRPM);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return lerp(
            floor.getValue(), 
            ceil.getValue(),
            (flywheelRPM - floor.getKey()) / (ceil.getKey() - floor.getKey())
        );
    }

    private static double lerp(double y1, double y2, double t) {
        return y1 + (t * (y2 - y1));
    }
    
}
