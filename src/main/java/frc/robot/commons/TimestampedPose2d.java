package frc.robot.commons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TimestampedPose2d extends Pose2d {

    private double timestamp = 0.0;

    public TimestampedPose2d(double timestamp) {
        super();
        this.timestamp = timestamp;
    }

    public TimestampedPose2d(double timestamp, Pose2d pose) {
        super(pose.getTranslation(), pose.getRotation());
        this.timestamp = timestamp;
    }

    public TimestampedPose2d(double timestamp, Translation2d translation, Rotation2d rotation) {
        super(translation, rotation);
        this.timestamp = timestamp;
    }

    public TimestampedPose2d(double timestamp, double x, double y, Rotation2d rotation) {
        super(x, y, rotation);
        this.timestamp = timestamp;
    }

    public double getAssociatedTimestamp() {
        return timestamp;
    }
    
}
