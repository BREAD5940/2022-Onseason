package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FakeRobot extends TimedRobot {

    XboxController controller = new XboxController(0);

    ChassisSpeeds fieldRelativeSpeeds;

    Pose2d fieldPose = new Pose2d();
    public final Field2d field = new Field2d();

    @Override
    public void robotInit() {
        super.robotInit();
        SmartDashboard.putData(field);
        setNetworkTablesFlushEnabled(true);
    }

    @Override
    public void robotPeriodic() {
        // X_k+1 = X_k + X-dot * dt
        double vx = -controller.getLeftY() * 3;
        double vy = -controller.getLeftX() * 3;
        double omega = controller.getRawAxis(2) * 5;

        var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, fieldPose.getRotation());

        fieldPose = fieldPose.plus(new Transform2d(
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                new Rotation2d(speeds.omegaRadiansPerSecond)).times(0.020));

        field.setRobotPose(fieldPose);
    }
}
