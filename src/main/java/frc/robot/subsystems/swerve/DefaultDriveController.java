package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Drive.*;

public class DefaultDriveController extends CommandBase {
    
    private final Swerve swerve;
    private final DoubleSupplier x, y, omega;

    public DefaultDriveController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, Swerve swerve) {
        this.swerve = swerve;
        this.x = x;
        this.y = y;
        this.omega = omega;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double dx = Math.abs(x.getAsDouble()) > 0.05 ? Math.pow(-x.getAsDouble(), 3) * ROBOT_MAX_SPEED * 0.7 : 0.0;
        double dy = Math.abs(y.getAsDouble()) > 0.05 ? Math.pow(-y.getAsDouble(), 3) * ROBOT_MAX_SPEED * 0.7 : 0.0;
        double rot = Math.abs(omega.getAsDouble()) > 0.05 ? Math.pow(-omega.getAsDouble(), 3) * 2 : 0.0;
        swerve.setSpeeds(dx, dy, rot);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setSpeeds(0.0, 0.0, 0.0);
    }

}
