package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;
import static frc.robot.Constants.Vision.*;

public class InterpolatingTable {

    private InterpolatingTable() {}

    public static TreeMap<Double, ShotParameter> table = new TreeMap<>(
        Map.ofEntries(
            entry(2.106467 + CAMERA_TO_CENTER, new ShotParameter(15.5, 1550.0)), // 43.5 inches
            entry(2.425149 + CAMERA_TO_CENTER, new ShotParameter(16.0, 1650.0)), // 55 inches
            entry(2.644736 + CAMERA_TO_CENTER, new ShotParameter(16.5, 1700.0)), // 64 inches
            entry(2.930702 + CAMERA_TO_CENTER, new ShotParameter(16.75, 1750.0)), // 75 inches
            entry(3.234261 + CAMERA_TO_CENTER, new ShotParameter(17.5, 1800.0)), // 85 inches
            entry(3.535450 + CAMERA_TO_CENTER, new ShotParameter(18.0, 1850.0)), // 95 inches
            entry(3.961900 + CAMERA_TO_CENTER, new ShotParameter(18.5, 1950.0)), // 110 inches
            entry(4.223835 + CAMERA_TO_CENTER, new ShotParameter(18.5, 2000.0)), // 120 inches
            entry(4.562416 + CAMERA_TO_CENTER, new ShotParameter(19.0, 2025.0)), // 130 inches 
            entry(4.799944 + CAMERA_TO_CENTER, new ShotParameter(19.5, 2050.0)), // 140 inches
            entry(5.128513 + CAMERA_TO_CENTER, new ShotParameter(20.0, 2100)), // 150 inches
            entry(5.431509 + CAMERA_TO_CENTER, new ShotParameter(20.5, 2200)) // 160 inches
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
