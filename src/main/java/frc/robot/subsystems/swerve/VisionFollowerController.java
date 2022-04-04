package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.MeanFilter;
import frc.robot.subsystems.vision.RobotPositionHistory;

public class VisionFollowerController extends CommandBase {

    private final MeanFilter headingReferenceFilter = new MeanFilter(3);
    private final Swerve swerve;
    private final ProfiledPIDController turnPID = new ProfiledPIDController(
        10 * (Math.PI/180.0), 0, 0.004,
        new TrapezoidProfile.Constraints(360.0, 720.0)
    );
    private double[] previousHeadingReferenceSample = new double[2]; // {previousHeadingReference, timestamp}

    public VisionFollowerController(Swerve swerve) { 
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        RobotPositionHistory.clear();
        turnPID.reset(swerve.getRawGyro());
        previousHeadingReferenceSample[0] = 0.0;
        previousHeadingReferenceSample[1] = RobotController.getFPGATime()/1.0E6;
    }

    @Override
    public void execute() {

        // Update the robot position history
        double currentHeading = swerve.getRawGyro();
        double currentTimestamp = RobotController.getFPGATime()/1.0E6;
        RobotPositionHistory.update(currentTimestamp, currentHeading);

        // Use vision readings to find the robot heading setpoint
        double yaw = RobotContainer.vision.getYaw();
        double visionTimestamp = RobotContainer.vision.getMeasurementTimestamp();
        double headingAtVisionTimestamp = RobotPositionHistory.get(visionTimestamp);
        double currentHeadingReference = headingAtVisionTimestamp + yaw;

        // Calculate the feed forward
        double unfilteredFF = (currentHeadingReference-previousHeadingReferenceSample[0])/(currentTimestamp-previousHeadingReferenceSample[1]);
        double FF = headingReferenceFilter.calculate(unfilteredFF);

        double omega = turnPID.calculate(currentHeading, currentHeadingReference);
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;
        swerve.setSpeeds(dx, dy, FF + omega);
        if (BreadUtil.atReference(currentHeading, currentHeadingReference, 2.0, true)) {
            SmartDashboard.putBoolean("At Vision Setpoint", true);
            swerve.setAtVisionHeadingSetpoint(true);
        } else {
            SmartDashboard.putBoolean("At Vision Setpoint", false);
            swerve.setAtVisionHeadingSetpoint(false);
        }
        SmartDashboard.putNumber("Error", currentHeadingReference - currentHeading);

        // Set the previous heading reference to be the current heading reference
        previousHeadingReferenceSample[0] = currentHeadingReference;
        previousHeadingReferenceSample[1] = currentTimestamp;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setAtVisionHeadingSetpoint(false);
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}
