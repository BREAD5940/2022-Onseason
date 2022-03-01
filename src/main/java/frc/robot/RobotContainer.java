package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.climber.DefaultClimberCommand;
import frc.robot.subsystems.intake.DualIntake;
import frc.robot.subsystems.intake.IntakePneumatics;
import frc.robot.subsystems.intake.IntakePneumaticsCommand;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Gut;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Neck;
import frc.robot.subsystems.swerve.DefaultDriveController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.climber.Climber;
import frc.robot.autonomus.routines.SixCargoSweep;
import frc.robot.autonomus.routines.TwoCargoTurnToRight;
import frc.robot.statemachines.Superstructure;

public class RobotContainer {

  Swerve swerve = new Swerve();
  DualIntake dualIntake = new DualIntake();
  IntakePneumatics intakePneumatics = new IntakePneumatics();
  Gut gut = new Gut();
  Neck neck = new Neck();
  Flywheel flywheel = new Flywheel();
  Hood hood = new Hood();
  Climber climber = new Climber();
  Superstructure superstructure = new Superstructure(dualIntake, gut, neck, flywheel, hood);
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public RobotContainer() {
    swerve.setDefaultCommand(new DefaultDriveController(driver::getRightY, driver::getRightX, driver::getLeftX, swerve));
    intakePneumatics.setDefaultCommand(new IntakePneumaticsCommand(intakePneumatics));
    climber.setDefaultCommand(new DefaultClimberCommand(climber));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kStart.value).whenPressed(
      new InstantCommand(() -> swerve.reset(new Pose2d()), swerve)
    );
  }

  public Command getAutonomousCommand() {
    return new TwoCargoTurnToRight(superstructure, intakePneumatics, swerve);
  }
}
