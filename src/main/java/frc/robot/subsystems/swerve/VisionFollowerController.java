package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
import frc.robot.interpolation.BallFlightTimeInterpolatingTable;
import frc.robot.interpolation.InterpolatingTable;
import frc.robot.interpolation.ShotParameter;
import frc.robot.subsystems.vision.RobotPositionHistory;
import static frc.robot.Constants.Vision.*;

public class VisionFollowerController extends CommandBase {

    private final Swerve swerve;
    private final PIDController turnPID = new PIDController(
        8, 0, 0
    );

    public VisionFollowerController(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.setTolerance(Units.degreesToRadians(2.0));
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {

        // Set the pose estimate to the latest vision measurement
        Pair<Pose2d, Double> timestampedVisionPoseEstimate = getLatestVisonPoseEstimate();
        Pose2d visionPoseEstimate = timestampedVisionPoseEstimate.getFirst();
        double associatedTimestamp = timestampedVisionPoseEstimate.getSecond();
        
        // Calculate pose relative to the target (factoring in wheel speeds)
        Pose2d currentAbsolutePose = swerve.getPose();
        SmartDashboard.putNumber("Vision Pose Estimate", associatedTimestamp);
        Pose2d absolutePoseAtVisionTimestamp = RobotPositionHistory.get(associatedTimestamp);
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
        double tangentialSpeed = targetRelativeVelocity.getY();
        double distanceToCenterOfHub = adjustedPoseEstimate.getTranslation().getNorm();
        double ff = -1 * tangentialSpeed / adjustedPoseEstimate.getTranslation().getNorm();

        // Adjusted Target Position
        double ballFlightTime = BallFlightTimeInterpolatingTable.get(distanceToCenterOfHub);
        Translation2d shotAimPosition = fieldRelativeVelocity.rotateBy(Rotation2d.fromDegrees(180.0)).times(ballFlightTime);

        // Get robot to adjusted target position
        Translation2d robotToShotAimPosition = shotAimPosition.minus(adjustedPoseEstimate.getTranslation());
        Rotation2d robotToShotAimPointAngle = new Rotation2d(robotToShotAimPosition.getX(), robotToShotAimPosition.getY());
        double robotToAdjustedTargetDistance = robotToShotAimPosition.getNorm();

        // Calculate (and apply) the shot parameter
        ShotParameter shot = InterpolatingTable.get(robotToAdjustedTargetDistance);
        RobotContainer.shooter.requestShoot(shot.flywheelRPM, shot.hoodAngleRadians);

        // Calculate the pid 
        double measurement = adjustedPoseEstimate.getRotation().getRadians();
        double setpoint = robotToShotAimPointAngle.getRadians();
        double pid = turnPID.calculate(measurement, setpoint);
        double clampAdd = 1 + Math.abs(setpoint - measurement) * (2/Math.PI);
        pid = MathUtil.clamp(pid, -clampAdd, clampAdd);

        SmartDashboard.putNumber("Vision Follower Distance", robotToAdjustedTargetDistance);

        // Handles x and y translation (manually controlled)
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;

        // Sets the speeds of the swerve drive 
        swerve.setSpeeds(dx, dy, ff + pid);

        if (turnPID.atSetpoint()) {
            swerve.setAtVisionHeadingSetpoint(true);
        } else {
            swerve.setAtVisionHeadingSetpoint(false);
        }

        SmartDashboard.putNumber("Vision Follower Setpoint", Units.radiansToDegrees(setpoint));
        SmartDashboard.putNumber("Vision Follower Measurement", Units.radiansToDegrees(measurement));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setAtVisionHeadingSetpoint(false);
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
    private Pair<Pose2d, Double> getLatestVisonPoseEstimate() {
        double yawDegrees = RobotContainer.vision.getYaw();
        double targetToCameraMeters = RobotContainer.vision.getCameraToCenterOfHub();
        double visionTimestampSeconds = RobotContainer.vision.getMeasurementTimestamp(); 
        Pose2d visionEstimatedPose = new Pose2d(
            -targetToCameraMeters, 0.0,
            Rotation2d.fromDegrees(-yawDegrees)
        );   
        Pose2d adjustedVisionEstimatePose = new Pose2d(
            visionEstimatedPose.getTranslation().plus(new Translation2d(-CAMERA_TO_CENTER, visionEstimatedPose.getRotation())),
            visionEstimatedPose.getRotation()
        );
        return new Pair<Pose2d, Double>(adjustedVisionEstimatePose, visionTimestampSeconds);
    }
    
}
