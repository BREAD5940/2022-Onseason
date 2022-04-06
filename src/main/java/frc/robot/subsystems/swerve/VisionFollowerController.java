package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.vision.RobotHeadingHistory;
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
        RobotHeadingHistory.clear();
        RobotPositionHistory.clear();
        swerve.resetPoseEstimator(new Pose2d(-(RobotContainer.vision.getDistance() + UPPER_HUB_RADIUS), 0.0, Rotation2d.fromDegrees(-RobotContainer.vision.getYaw())));
        // previousHeadingReferenceSample[0] = 0.0;
        // previousHeadingReferenceSample[1] = RobotController.getFPGATime()/1.0E6;
    }

    @Override
    public void execute() {

        double yaw = RobotContainer.vision.getYaw();
        double visionTimestamp = RobotContainer.vision.getMeasurementTimestamp();

        // Update odometry
        Pose2d currentPoseEstimate = swerve.getEstimatedPose(); 
        RobotPositionHistory.update(RobotController.getFPGATime()/1.0E6, currentPoseEstimate);
        Pose2d goalToRobotEstimate = RobotPositionHistory.get(visionTimestamp);
        Translation2d robotToGoalTranslation = goalToRobotEstimate.getTranslation().rotateBy(Rotation2d.fromDegrees(180.0));
        Rotation2d robotToGoalAngle = new Rotation2d(robotToGoalTranslation.getX(), robotToGoalTranslation.getY());
        double poseEstimatedDistance = goalToRobotEstimate.getTranslation().getNorm();
        double visionEstimatedDistace = RobotContainer.vision.getDistance() + UPPER_HUB_RADIUS; 
        Pose2d adjustedPose = new Pose2d(
            goalToRobotEstimate.getTranslation().times(visionEstimatedDistace/poseEstimatedDistance),
            robotToGoalAngle.minus(Rotation2d.fromDegrees(yaw))
        );
        swerve.addVisionMeasurement(adjustedPose, visionTimestamp);
        swerve.field.setRobotPose(new Pose2d(
            currentPoseEstimate.getTranslation().plus(FIELD_TO_TARGET),
            currentPoseEstimate.getRotation()
        ));                             

        // Calculate Feed Forward
        Rotation2d currentRobotToGoalAngle = new Rotation2d(currentPoseEstimate.getX(), currentPoseEstimate.getY()).rotateBy(Rotation2d.fromDegrees(180.0));
        Translation2d currentRobotToGoalTranslation = currentPoseEstimate.getTranslation().rotateBy(Rotation2d.fromDegrees(180.0));
        Translation2d robotRelativeVelocity = swerve.getVelocity();
        Translation2d fieldRelativeVelocity = robotRelativeVelocity.rotateBy(currentPoseEstimate.getRotation());
        Translation2d velocityTowardsTarget = BreadUtil.vectorProjection(fieldRelativeVelocity, currentRobotToGoalTranslation);
        Translation2d velocityTangentToTarget = fieldRelativeVelocity.minus(velocityTowardsTarget);
        double tangentSpeed = velocityTangentToTarget.rotateBy(currentRobotToGoalAngle.times(-1)).getY();
        double ff = -1 * tangentSpeed / visionEstimatedDistace;

        SmartDashboard.putNumber("Current Robot To Goal Angle", currentRobotToGoalAngle.getDegrees());
        SmartDashboard.putNumber("Current Robot Angle", currentPoseEstimate.getRotation().getDegrees());

        SmartDashboard.putNumber("Velocity Tangent To Target Angle", new Rotation2d(velocityTangentToTarget.getX(), velocityTangentToTarget.getY()).getDegrees());
        SmartDashboard.putNumber("Velocity Towards Target Angle", new Rotation2d(velocityTowardsTarget.getX(), velocityTowardsTarget.getY()).getDegrees());

        // Calculate PID
        double pid = turnPID.calculate(currentPoseEstimate.getRotation().getDegrees(), currentRobotToGoalAngle.getDegrees());
               
        // Turn to face the target
        // double pid = turnPID.calculate(currentHeading, currentHeadingReference);
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;
        swerve.setSpeeds(dx, dy, ff + pid);

        SmartDashboard.putNumber("Measurement Drive", currentPoseEstimate.getRotation().getDegrees());
        SmartDashboard.putNumber("Setpoint Drive", currentRobotToGoalAngle.getDegrees());
        // SmartDashboard.putNumber("Tangent Velocity Vector", velocityTangentToTarget.getY());
        // if (BreadUtil.atReference(currentHeading, currentHeadingReference, 2.0, true)) {
        //     SmartDashboard.putBoolean("At Vision Setpoint", true);
        //     swerve.setAtVisionHeadingSetpoint(true);
        // } else {
        //     SmartDashboard.putBoolean("At Vision Setpoint", false);
        //     swerve.setAtVisionHeadingSetpoint(false);
        // }
        // SmartDashboard.putNumber("Error", currentHeadingReference - currentHeading);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setAtVisionHeadingSetpoint(false);
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}
