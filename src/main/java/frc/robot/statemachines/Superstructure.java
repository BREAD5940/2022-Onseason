package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.DualIntake;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Gut;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Neck;
import static frc.robot.Constants.Hood.*;

public class Superstructure extends SubsystemBase {

    public final DualIntake dualIntake;
    public final Gut gut;
    public final Neck neck;
    public final Flywheel flywheel;
    public final Hood hood;

    public Superstructure(DualIntake dualIntake, Gut gut, Neck neck, Flywheel flywheel, Hood hood) {
        this.dualIntake = dualIntake;
        this.gut = gut;
        this.neck = neck;
        this.flywheel = flywheel;
        this.hood = hood;
    }

    public void idle() {
        gut.setSurfaceSpeed(0.0);
        neck.setSurfaceSpeed(0.0);
        dualIntake.setLeft(0.0);
        dualIntake.setRight(0.0);
        if (RobotContainer.controller.getRightStickButton())
            flywheel.setVelocity(3000.0);
        else 
            flywheel.setVelocity(0.0);
        hood.setPosition(HOOD_IDLE_POS);
    }

    public void intakeLeft() {
        dualIntake.setLeft(0.8);
        dualIntake.setRight(-0.8);
        gut.setSurfaceSpeed(2.0);
        neck.setSurfaceSpeed(0.0);
        if (RobotContainer.controller.getRightStickButton())
            flywheel.setVelocity(3000.0);
        else 
            flywheel.setVelocity(0.0);
        hood.setPosition(HOOD_IDLE_POS);
    }

    public void intakeRight() {
        dualIntake.setLeft(-0.8);
        dualIntake.setRight(0.8);
        gut.setSurfaceSpeed(-2.0);
        neck.setSurfaceSpeed(0.0);
        if (RobotContainer.controller.getRightStickButton())
            flywheel.setVelocity(3000.0);
        else 
            flywheel.setVelocity(0.0);
        hood.setPosition(HOOD_IDLE_POS);
    }

    public void stowCargoInNeck() {
        dualIntake.setLeft(0.0);
        dualIntake.setRight(0.0);
        gut.setSurfaceSpeed(0.0);
        neck.setSurfaceSpeed(2.0);
        if (RobotContainer.controller.getRightStickButton())
            flywheel.setVelocity(3000.0);
        else 
            flywheel.setVelocity(0.0);
        hood.setPosition(HOOD_IDLE_POS);
    }

    public void spinup() {
        dualIntake.setLeft(0.0);
        dualIntake.setRight(0.0);
        gut.setSurfaceSpeed(0.0);
        neck.setSurfaceSpeed(0.0);
        double flywheelVelocity = SmartDashboard.getNumber("Shooter Setpoint", 0.0);
        flywheel.setVelocity(flywheelVelocity);
        double hoodAngle = SmartDashboard.getNumber("Hood Setpoint", 0.0);
        hood.setPosition(hoodAngle);
    }

    public void shoot() {
        dualIntake.setLeft(0.0);
        dualIntake.setRight(0.0);
        gut.setSurfaceSpeed(0.0);
        neck.setSurfaceSpeed(1.0);
        double flywheelVelocity = SmartDashboard.getNumber("Shooter Setpoint", 0.0);
        flywheel.setVelocity(flywheelVelocity);
        double hoodAngle = SmartDashboard.getNumber("Hood Setpoint", 0.0);
        hood.setPosition(hoodAngle);
    }
}
