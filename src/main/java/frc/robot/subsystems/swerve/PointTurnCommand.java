package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commons.BreadUtil;

public class PointTurnCommand extends CommandBase {

    private final ProfiledPIDController turnPID = new ProfiledPIDController(
        9, 0, 0.004,
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(360.0), 
            Units.degreesToRadians(720.0)
        )
    );
    Swerve swerve;
    DoubleSupplier headingSupplier;
    
    public PointTurnCommand(DoubleSupplier headingSupplier, Swerve swerve) {
        this.swerve = swerve;
        this.headingSupplier = headingSupplier;
        addRequirements(swerve);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        turnPID.reset(swerve.getMatchPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        swerve.setSpeeds(0.0, 0.0, turnPID.calculate(swerve.getMatchPose().getRotation().getRadians(), headingSupplier.getAsDouble()));
        System.out.printf("Current: %.2f, Setpoint: %.2f, Error: %.2f\n",
            swerve.getMatchPose().getRotation().getDegrees(),
            Units.radiansToDegrees(headingSupplier.getAsDouble()),
            Units.radiansToDegrees(headingSupplier.getAsDouble())-swerve.getMatchPose().getRotation().getDegrees()
        );
    }

    @Override
    public boolean isFinished() {
        return BreadUtil.atReference(swerve.getMatchPose().getRotation().getRadians(), headingSupplier.getAsDouble(), Units.degreesToRadians(2.0), true);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>>>>>>> point turn command finished");
        swerve.setSpeeds(0.0, 0.0, 0.0);
    }

    
    
}
