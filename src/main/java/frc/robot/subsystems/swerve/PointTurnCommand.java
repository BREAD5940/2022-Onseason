package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointTurnCommand extends CommandBase {

    PIDController pid = new PIDController(8, 0, 0);
    Swerve swerve;
    DoubleSupplier headingSupplier;
    
    public PointTurnCommand(DoubleSupplier headingSupplier, Swerve swerve) {
        this.swerve = swerve;
        this.headingSupplier = headingSupplier;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setSpeeds(0.0, 0.0, pid.calculate(swerve.getPose().getRotation().getRadians(), headingSupplier.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return pid.getPositionError() < 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSpeeds(0.0, 0.0, 0.0);
    }

    
    
}
