package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointTurnCommand extends CommandBase {

    private final PIDController turnPID = new PIDController(9, 0, 0.004);
    Swerve swerve;
    DoubleSupplier headingSupplier;
    
    public PointTurnCommand(DoubleSupplier headingSupplier, Swerve swerve) {
        this.swerve = swerve;
        this.headingSupplier = headingSupplier;
        addRequirements(swerve);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.setTolerance(Units.degreesToRadians(2.0));    
    }

    @Override
    public void execute() {
        swerve.setSpeeds(0.0, 0.0, MathUtil.clamp(turnPID.calculate(swerve.getPose().getRotation().getRadians(), headingSupplier.getAsDouble()), -2, 2));
        // System.out.printf("Current: %.2f, Setpoint: %.2f, Error: %.2f\n",
        //     swerve.getPose().getRotation().getDegrees(),
        //     Units.radiansToDegrees(headingSupplier.getAsDouble()),
        //     Units.radiansToDegrees(headingSupplier.getAsDouble())-swerve.getPose().getRotation().getDegrees()
        // );
    }

    @Override
    public boolean isFinished() {
        return turnPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>>>>>>> point turn command finished");
        swerve.setSpeeds(0.0, 0.0, 0.0);
    }

    
    
}
