
package frc.robot.subsystems.swerve;

import java.util.function.BiFunction;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commons.BreadHolonomicDriveController;

public class TrajectoryFollowerController extends CommandBase {

    private final Trajectory trajectory;
    private final BiFunction<Pose2d, Double, Rotation2d> refHeading;
    private final Supplier<Rotation2d> startHeading;
    private final Swerve swerve;
    private final Timer timer = new Timer();
    public final BreadHolonomicDriveController autonomusController = new BreadHolonomicDriveController(
        new PIDController(8, 0, 0), 
        new PIDController(8, 0, 0), 
        new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints( // P was 6 at madtown field
            Units.degreesToRadians(360.0),
            Units.degreesToRadians(180.0)
        ))
    );

    public TrajectoryFollowerController(Trajectory trajectory, BiFunction<Pose2d, Double, Rotation2d> refHeading, Supplier<Rotation2d> startHeading, Swerve swerve) {
        this.trajectory = trajectory;
        this.refHeading = refHeading;
        this.startHeading = startHeading;
        this.swerve = swerve;
        addRequirements(swerve);
    }

    public TrajectoryFollowerController(Trajectory trajectory, BiFunction<Pose2d, Double, Rotation2d> refHeading, Swerve swerve) {
        this(trajectory, refHeading, null, swerve);
    }

    @Override
    public void initialize() {
        if (startHeading != null) swerve.reset(new Pose2d(trajectory.sample(0.0).poseMeters.getTranslation(), startHeading.get()));
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = autonomusController.calculate(swerve.getPose(), goal, refHeading.apply(swerve.getPose(), timer.get())); 
        swerve.setSpeeds(
            adjustedSpeeds
        );
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }

    @Override
    public void end(boolean interrupted) { 
        swerve.setSpeeds(new ChassisSpeeds(0, 0, 0));
    }

}
