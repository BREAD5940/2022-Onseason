package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;
import static frc.robot.Constants.Vision.*;

public class InterpolatingTable {

    private InterpolatingTable() {}

    public static TreeMap<Double, ShotParameter> table = new TreeMap<>(
        // Map.ofEntries(
        //     entry(1.654, new ShotParameter(18, 1500)), // 44 inches to front of bumper from hub wall
        //     entry(2.32, new ShotParameter(20, 1600)), // 68 inches to front of bumper from hub wall
        //     entry(2.649, new ShotParameter(21.5, 1700)), // 80 inches
        //     entry(3.163, new ShotParameter(21.5, 1775)), // 98 inches
        //     entry(3.72, new ShotParameter(22.5, 1850)), // 117 inches
        //     entry(4.264, new ShotParameter(24, 1950)), // 136 inches
        //     entry(4.86, new ShotParameter(25.5, 2100)) // 156 inches
        // )
        // Map.ofEntries(
        //     entry(1.409, new ShotParameter(17, 1450)), // 39 inches
        //     entry(1.740, new ShotParameter(18, 1500)), // 53 inches
        //     entry(1.976, new ShotParameter(19, 1525)), // 63 inches
        //     entry(2.1919, new ShotParameter(20.5, 1550)), // 71 inches
        //     entry(2.400, new ShotParameter(22, 1600)), // 80 inches
        //     entry(2.618, new ShotParameter(23, 1650)), // 88 inches
        //     entry(2.895, new ShotParameter(23.5, 1700)), // 99 inches
        //     entry(3.139, new ShotParameter(24, 1750)), // 110 inches
        //     entry(3.378, new ShotParameter(24.5, 1800)), // 121 inches
        //     entry(3.746, new ShotParameter(25, 1875)), // 136 inches
        //     entry(4.060, new ShotParameter(25, 1950)) // 148 inches 
        // )
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
