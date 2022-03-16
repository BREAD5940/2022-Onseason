package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.RobotPositionHistory;

public class VisionTurnCommand extends CommandBase {

    private final Swerve swerve;
    private final PIDController turnPID = new PIDController(0.1, 0, 0);

    public VisionTurnCommand(Swerve swerve) { 
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        RobotPositionHistory.clear();
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = swerve.getPose().getRotation();
        RobotPositionHistory.update(RobotController.getFPGATime()/1.0E6, swerve.getPose().getRotation());
        double yaw = RobotContainer.vision.getYaw();
        double visionTimestamp = RobotContainer.vision.getMeasurementTimestamp();
        Rotation2d headingAtVisionTimestamp = RobotPositionHistory.get(visionTimestamp);
        double error = yaw - (currentHeading.minus(headingAtVisionTimestamp).getDegrees());
        double output = turnPID.calculate(error, 0.0);
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, -output));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}
