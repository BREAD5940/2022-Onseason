package frc.robot.subsystems.swerve;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class DefaultDriveController extends CommandBase {
    
    private final Swerve swerve;
    private final SlewRateLimiter xLimit = new SlewRateLimiter(1/.15);
    private final SlewRateLimiter yLimit = new SlewRateLimiter(1/.15);
    private final DoubleSupplier speedSupplier;

    public DefaultDriveController(Swerve swerve, DoubleSupplier speedSupplier) {
        this.swerve = swerve;
        this.speedSupplier = speedSupplier;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double x = xLimit.calculate(RobotContainer.driver.getRightY());
        double y = yLimit.calculate(RobotContainer.driver.getRightX());
        double omega = RobotContainer.driver.getLeftX();
        double dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) * speedSupplier.getAsDouble() : 0.0;
        double dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) * speedSupplier.getAsDouble(): 0.0;
        double rot = Math.abs(omega) > 0.05 ? Math.pow(-omega, 3) * 2 : 0.0;
        swerve.setSpeeds(dx, dy, rot);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setSpeeds(0.0, 0.0, 0.0);
    }

}
