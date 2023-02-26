package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swerve.DefaultDriveController;
import frc.robot.subsystems.swerve.Swerve;


public class RobotContainer {
  public static Swerve swerve = new Swerve();
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);

  public RobotContainer() {
    swerve.setDefaultCommand(new DefaultDriveController(swerve));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kStart.value).whenPressed(
      new InstantCommand(() -> swerve.reset(new Pose2d()), swerve)
    );
  }
}