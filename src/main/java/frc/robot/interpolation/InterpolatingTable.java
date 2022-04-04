package frc.robot.interpolation;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;
import static java.util.Map.entry;

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
        Map.ofEntries(
            entry(1.409, new ShotParameter(17, 1450)), // 39 inches
            entry(1.740, new ShotParameter(18, 1500)), // 53 inches
            entry(1.976, new ShotParameter(19, 1525)), // 63 inches
            entry(2.1919, new ShotParameter(20.5, 1550)), // 71 inches
            entry(2.400, new ShotParameter(22, 1600)), // 80 inches
            entry(2.618, new ShotParameter(23, 1650)), // 88 inches
            entry(2.895, new ShotParameter(23.5, 1700)), // 99 inches
            entry(3.139, new ShotParameter(24, 1750)), // 110 inches
            entry(3.378, new ShotParameter(24.5, 1800)), // 121 inches
            entry(3.746, new ShotParameter(25, 1875)), // 136 inches
            entry(4.060, new ShotParameter(25, 1950)) // 148 inches 
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
