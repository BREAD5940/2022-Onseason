package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commons.TimestampedPose2d;
import frc.robot.subsystems.vision.RobotPositionHistory;
import static frc.robot.Constants.Drive.*;

public class VisionFollowerController extends CommandBase {

    private final Swerve swerve;
    private final PIDController turnPID = new PIDController(
        10 * (Math.PI/180.0), 0, 0.004
    );

    public VisionFollowerController(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        RobotPositionHistory.clear();
    }

    @Override
    public void execute() {

        // Set the pose estimate to the latest vision measurement
        TimestampedPose2d visionPoseEstimate = getLatestVisonPoseEstimate();
        
        // Calculate pose relative to the target (factoring in wheel speeds)
        Pose2d currentAbsolutePose = swerve.getPose();
        Pose2d absolutePoseAtVisionTimestamp = RobotPositionHistory.get(visionPoseEstimate.getAssociatedTimestamp());
        Transform2d changeInPose = currentAbsolutePose.minus(absolutePoseAtVisionTimestamp);
        changeInPose = new Transform2d(
            changeInPose.getTranslation().rotateBy(visionPoseEstimate.getRotation().minus(currentAbsolutePose.getRotation())), 
            changeInPose.getRotation()
        );
        Pose2d adjustedPoseEstimate = visionPoseEstimate.transformBy(changeInPose); 

        // Calculate the feed forward
        Translation2d fieldRelativeVelocity = swerve.getVelocity().rotateBy(adjustedPoseEstimate.getRotation());
        Rotation2d robotToGoalAngle = new Rotation2d(adjustedPoseEstimate.getX(), adjustedPoseEstimate.getY()).rotateBy(Rotation2d.fromDegrees(180.0));
        Translation2d targetRelativeVelocity = fieldRelativeVelocity.rotateBy(robotToGoalAngle.times(-1));
        double tangentSpeed = targetRelativeVelocity.getY();
        double ff = tangentSpeed / adjustedPoseEstimate.getTranslation().getNorm();

        // Calculate the pid 
        double pid = turnPID.calculate(adjustedPoseEstimate.getRotation().getDegrees(), robotToGoalAngle.getDegrees());

        // Handles x and y translation (manually controlled)
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;

        // Sets the speeds of the swerve drive 
        swerve.setSpeeds(dx, dy, ff + pid);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setAtVisionHeadingSetpoint(false);
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private TimestampedPose2d getLatestVisonPoseEstimate() {
        double yawDegrees = RobotContainer.vision.getYaw();
        double targetToCameraMeters = RobotContainer.vision.getDistance() + Units.inchesToMeters(UPPER_HUB_RADIUS);
        double visionTimestampSeconds = RobotContainer.vision.getMeasurementTimestamp(); 
        Pose2d visionEstimatedPose = new Pose2d(
            -targetToCameraMeters, 0.0,
            Rotation2d.fromDegrees(-yawDegrees)
        );   
        return new TimestampedPose2d(visionTimestampSeconds, visionEstimatedPose);
    }
    
}
