package frc.robot.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.swerve.VisionFollowerController;

public class FakeRobot extends TimedRobot {

    XboxController controller = new XboxController(0);

    Pose2d fieldPose = new Pose2d();
    Pose2d fieldPose2 = new Pose2d();
    public final Field2d field = new Field2d();

    @Override
    public void robotInit() {
        super.robotInit();
        SmartDashboard.putData(field);
        setNetworkTablesFlushEnabled(true);
        field.getObject("Goal").setPose(new Pose2d(4, 4, new Rotation2d()));
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        turnPID2.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final PIDController turnPID = new PIDController(
            10, 0, 0
    );

    private final PIDController turnPID2 = new PIDController(
        10, 0, 0
    );

    @Override
    public void robotPeriodic() {
        // X_k+1 = X_k + X-dot * dt
        double vy = -controller.getLeftY() * 3;
        double vx = controller.getLeftX() * 3;

        var omega = VisionFollowerController.getFeedforward(new Translation2d(vx, vy), fieldPose);

        Rotation2d robotToGoalAngle = new Rotation2d(fieldPose.getX(), fieldPose.getY()).rotateBy(Rotation2d.fromDegrees(180.0));
        double measurement = fieldPose.getRotation().getRadians();
        double setpoint = robotToGoalAngle.minus(getSetpoint2(new Translation2d(vx, vy), fieldPose)).getRadians();
        double pid = turnPID.calculate(measurement, setpoint);
        omega += pid;
        omega += controller.getRawAxis(2) * 5;

        var omega2 = VisionFollowerController.getFeedforward(new Translation2d(vx, vy), fieldPose);
        double measurement2 = fieldPose2.getRotation().getRadians();
        double setpoint2 = getSetpoint1(new Translation2d(vx, vy), fieldPose2).getRadians();
        double pid2 = turnPID.calculate(measurement2, setpoint2);
        omega2 += pid2;
        omega += controller.getRawAxis(2) * 5;

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, fieldPose.getRotation());
        var speeds2 = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega2, fieldPose2.getRotation());

        fieldPose = fieldPose.plus(new Transform2d(
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                new Rotation2d(speeds.omegaRadiansPerSecond)).times(0.020));

        // Field Pose 2
        fieldPose2 = fieldPose2.plus(
            new Transform2d(
                new Translation2d(speeds2.vxMetersPerSecond, speeds2.vyMetersPerSecond),
                new Rotation2d(speeds2.omegaRadiansPerSecond)).times(0.020)
        );

        var pose = new Pose2d(fieldPose.getX() + 4, fieldPose.getY() + 4, fieldPose.getRotation());
        field.setRobotPose(pose);

        field.getObject("Robot2").setPose(
            new Pose2d(
                fieldPose2.getX() + 4, fieldPose2.getY() + 4, fieldPose2.getRotation()
            )
        );

        // field.getObject("Robot2").setPose(new Pose2d(
        //     fieldPose2.getX() + 4, 
        //     fieldPose2.getY() + 4,
        //     fieldPose2.getRotation()
        // ));

        SmartDashboard.putNumber("Current", Math.abs(fieldPose2.getRotation().minus(robotToGoalAngle).getDegrees()));
        SmartDashboard.putNumber("Old", Math.abs(fieldPose.getRotation().minus(robotToGoalAngle).getDegrees()));

    }


    private Rotation2d getSetpoint2(Translation2d velocityInRobotRelativeFrame, Pose2d adjustedPose) {
        Translation2d fieldRelativeVelocity = velocityInRobotRelativeFrame;
        Rotation2d robotToGoalAngle = new Rotation2d(adjustedPose.getX(), adjustedPose.getY()).rotateBy(Rotation2d.fromDegrees(180.0));
        Translation2d targetRelativeVelocity = fieldRelativeVelocity.rotateBy(robotToGoalAngle.times(-1));
        double tangentialSpeed = targetRelativeVelocity.getY(); 

        double ballFlightTime = 1.23;

        return new Rotation2d(Math.atan(tangentialSpeed*ballFlightTime/adjustedPose.getTranslation().getNorm()));
    }

    private Rotation2d getSetpoint1(Translation2d velocityInRobotRelativeFrame, Pose2d adjustedPose) {
        Translation2d fieldRelativeVelocity = velocityInRobotRelativeFrame;
        Rotation2d robotToGoalAngle = new Rotation2d(adjustedPose.getX(), adjustedPose.getY()).rotateBy(Rotation2d.fromDegrees(180.0));
        Translation2d targetRelativeVelocity = fieldRelativeVelocity.rotateBy(robotToGoalAngle.times(-1));
        double tangentialSpeed = targetRelativeVelocity.getY();
        double distanceToCenterOfHub = adjustedPose.getTranslation().getNorm();
        double ff = -1 * tangentialSpeed / adjustedPose.getTranslation().getNorm();

        // Adjusted Target Position
        double ballFlightTime = 1.23;
        Translation2d shotAimPosition = fieldRelativeVelocity.rotateBy(Rotation2d.fromDegrees(180.0)).times(ballFlightTime);

        // Get robot to adjusted target position
        Translation2d robotToShotAimPosition = shotAimPosition.minus(adjustedPose.getTranslation());
        Rotation2d robotToShotAimPointAngle = new Rotation2d(robotToShotAimPosition.getX(), robotToShotAimPosition.getY());
        double robotToAdjustedTargetDistance = robotToShotAimPosition.getNorm();

        return robotToShotAimPointAngle;
    }
}
