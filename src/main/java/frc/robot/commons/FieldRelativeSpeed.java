package frc.robot.commons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldRelativeSpeed {
    public double vx;
    public double vy;
    public double omega;

    public FieldRelativeSpeed(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public FieldRelativeSpeed(ChassisSpeeds speeds, Rotation2d gyro) {
        this(
            speeds.vxMetersPerSecond * gyro.getCos() - speeds.vyMetersPerSecond * gyro.getSin(),
            speeds.vyMetersPerSecond * gyro.getCos() + speeds.vxMetersPerSecond * gyro.getSin(),
            speeds.omegaRadiansPerSecond
        );
    }

    public FieldRelativeSpeed() {
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;
    }
}