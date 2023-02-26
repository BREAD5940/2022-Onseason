package frc.robot;

import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private double lastResetToAbsolute = 0.0;

  @Override
  public void robotInit() {
    setUseTiming(isReal());
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard");
    Logger.getInstance().recordMetadata("ProjectName", "2022-Onseason");

    if (isReal()) {
      Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda2/"));
      Logger.getInstance().addDataReceiver(new LogSocketServer(5800));
    } else {
      String path = ByteLogReplay.promptForPath();
      Logger.getInstance().setReplaySource(new ByteLogReplay(path));
      Logger.getInstance().addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim")));
    }

    Logger.getInstance().start();
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
    lastResetToAbsolute = RobotController.getFPGATime() / 1.0E6;
  }

  @Override
  public void disabledPeriodic() {
    if ((RobotController.getFPGATime() / 1.0E6) - lastResetToAbsolute > 1.0) {
      RobotContainer.swerve.resetAllToAbsolute();
      lastResetToAbsolute = RobotController.getFPGATime() / 1.0E6;
    }
  }

  @Override
  public void autonomousInit() {
    // Set alliance color
    RobotContainer.swerve.setDriveSlots(1);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Set alliance color

    RobotContainer.swerve.setDriveSlots(0);

    RobotContainer.operator.getAButtonPressed();
    RobotContainer.operator.getYButtonPressed();
  }

  @Override
  public void teleopPeriodic() {
    configureTeleopControls();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  // Method to handle teleoperated controls
  public void configureTeleopControls() {
    // Driver Max Speeds
    if (RobotContainer.driver.getLeftTriggerAxis() > 0.1 || RobotContainer.driver.getRightTriggerAxis() > 0.1) {
      RobotContainer.swerve.defaultDriveSpeed = 3.75;
    } else {
      RobotContainer.swerve.defaultDriveSpeed = 4.5;
    }
  }

}
