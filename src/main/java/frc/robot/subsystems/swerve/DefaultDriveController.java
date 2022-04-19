package frc.robot.subsystems.swerve;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class DefaultDriveController extends CommandBase {
    
    private final Swerve swerve;
    private final SlewRateLimiter xLimit = new SlewRateLimiter(1/.05);
    private final SlewRateLimiter yLimit = new SlewRateLimiter(1/.05);

    public DefaultDriveController(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
        double omega = RobotContainer.driver.getLeftX();
        double dx = xLimit.calculate(Math.abs(x) > 0.05 ? Math.pow(-x, 1) * swerve.defaultDriveSpeed : 0.0);
        double dy = yLimit.calculate(Math.abs(y) > 0.05 ? Math.pow(-y, 1) * swerve.defaultDriveSpeed : 0.0);
        double rot = Math.abs(omega) > 0.05 ? Math.pow(-omega, 3) * 2 : 0.0;
        swerve.setSpeeds(dx, dy, rot);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setSpeeds(0.0, 0.0, 0.0);
    }

}
