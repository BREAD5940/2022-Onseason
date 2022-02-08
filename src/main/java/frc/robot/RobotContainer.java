package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.DefaultDriveController;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

  Swerve swerve = new Swerve();
  XboxController controller = new XboxController(0);

  public RobotContainer() {
    swerve.setDefaultCommand(new DefaultDriveController(controller::getRightY, controller::getRightX, controller::getLeftX, swerve));
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
