package frc.robot.interpolation;

public class ShotParameter {

    public final double hoodAngleRadians;
    public final double flywheelRPM;

    public ShotParameter(double hoodAngleRadians, double flywheelRPM) {
        this.hoodAngleRadians = hoodAngleRadians;
        this.flywheelRPM = flywheelRPM;
    }

    public boolean equals(ShotParameter other) {
        return Math.abs(other.hoodAngleRadians - hoodAngleRadians) < 0.1 && 
        Math.abs(other.flywheelRPM - flywheelRPM) < 0.1;
    }

    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(hoodAngleRadians, end.hoodAngleRadians, t), 
            lerp(flywheelRPM, end.flywheelRPM, t)
        );
    }

    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
    
}
