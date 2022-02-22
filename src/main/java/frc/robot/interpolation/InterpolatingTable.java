package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

public class InterpolatingTable {

    private InterpolatingTable() {}

    public static TreeMap<Double, ShotParameter> table = new TreeMap<>(
        Map.ofEntries(
            entry(0.0, new ShotParameter(0, 0))
        )
    );

    public static ShotParameter get(double distanceToTarget) {
        Entry<Double, ShotParameter> ceil = table.ceilingEntry(distanceToTarget);
        Entry<Double, ShotParameter> floor = table.floorEntry(distanceToTarget);
        if (ceil == null) return floor.getValue();
        if (floor == null) return ceil.getValue();
        if (ceil.getValue().equals(floor.getValue())) return ceil.getValue();
        return floor.getValue().interpolate(
            ceil.getValue(), 
            (distanceToTarget - floor.getKey()) / (ceil.getKey() - floor.getKey())
        );
    }
    
}
