package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class VisionTurnCommand extends CommandBase {

    private final Swerve swerve;
    private final PIDController turnPID = new PIDController(1, 0, 0);

    public VisionTurnCommand(Swerve swerve) { 
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double output = turnPID.calculate(RobotContainer.vision.getYaw());
        swerve.setSpeeds(0.0, 0.0, output);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    
}
