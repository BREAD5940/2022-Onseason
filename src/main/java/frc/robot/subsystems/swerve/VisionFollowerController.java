package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        Rotation2d robotToGoalAngle = new Rotation2d(adjustedPoseEstimate.getX(), adjustedPoseEstimate.getY()).rotateBy(Rotation2d.fromDegrees(180.0));

        // Calculate the pid 
        double pid = turnPID.calculate(adjustedPoseEstimate.getRotation().getDegrees(), robotToGoalAngle.getDegrees());

        double ff = getFeedforward(swerve.getVelocity(), adjustedPoseEstimate);

        // Handles x and y translation (manually controlled)
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;

        // Sets the speeds of the swerve drive 
        swerve.setSpeeds(dx, dy, ff + pid);
    }

    public static double getFeedforward(Translation2d velocityInRobotFrame, Pose2d adjustedPoseEstimate) {
        // Calculate the feed forward
        // Translation2d fieldRelativeVelocity = velocityInRobotFrame.rotateBy(adjustedPoseEstimate.getRotation());
        Translation2d fieldRelativeVelocity = velocityInRobotFrame;
        Rotation2d robotToGoalAngle = new Rotation2d(adjustedPoseEstimate.getX(), adjustedPoseEstimate.getY()).rotateBy(Rotation2d.fromDegrees(180.0));
        Translation2d targetRelativeVelocity = fieldRelativeVelocity.rotateBy(robotToGoalAngle.times(-1));
        double tangentSpeed = targetRelativeVelocity.getY();

        SmartDashboard.putNumber("Field Relative DX", fieldRelativeVelocity.getX());
        SmartDashboard.putNumber("Field Relative DY", fieldRelativeVelocity.getY());

        return -1 * tangentSpeed / adjustedPoseEstimate.getTranslation().getNorm();
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
