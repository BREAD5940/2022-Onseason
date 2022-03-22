package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.vision.RobotPositionHistory;
import static frc.robot.Constants.Drive.*;

public class VisionTurnCommand extends CommandBase {

    private final Swerve swerve;
    private final ProfiledPIDController turnPID = new ProfiledPIDController(
        5 * (Math.PI/180.0), 0, 0.004,
        new TrapezoidProfile.Constraints(360.0, 720.0)
    );

    public VisionTurnCommand(Swerve swerve) { 
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        RobotPositionHistory.clear();
        turnPID.reset(swerve.getRawGyro());
    }

    @Override
    public void execute() {
        double currentHeading = swerve.getRawGyro();
        RobotPositionHistory.update(RobotController.getFPGATime()/1.0E6, currentHeading);
        double yaw = RobotContainer.vision.getYaw();
        double visionTimestamp = RobotContainer.vision.getMeasurementTimestamp();
        double headingAtVisionTimestamp = RobotPositionHistory.get(visionTimestamp);
        double headingSetpoint = headingAtVisionTimestamp + yaw;
        double omega = turnPID.calculate(currentHeading, headingSetpoint);
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;
        swerve.setSpeeds(dx, dy, omega);
        if (BreadUtil.atReference(currentHeading, headingSetpoint, 2.0, true)) {
            swerve.setAtVisionHeadingSetpoint(true);
        } else {
            swerve.setAtVisionHeadingSetpoint(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setAtVisionHeadingSetpoint(false);
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}
