package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sensors.ColorSensor.BallColor;
import static frc.robot.Constants.Hood.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private boolean homed = false;
  public static BallColor allianceColor = BallColor.NONE;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putString("State", "NA");
    SmartDashboard.putNumber("Shooter Setpoint", 1350.0);
    SmartDashboard.putNumber("Hood Setpoint", 8.5);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    if (!homed) {
      RobotContainer.shooter.requestHome();
      homed = true;
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (!homed) {
      RobotContainer.shooter.requestHome();
      homed = true;
    }
    
    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;
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
  public void testPeriodic() {}

  // Method to handle teleoperated controls
  public void configureTeleopControls() {
       if (RobotContainer.operator.getLeftStickButton()) {
            RobotContainer.shooter.requestShoot(1350.0, HOOD_IDLE_POS);
        } else {
            RobotContainer.shooter.requestIdle();
        }
        if (RobotContainer.operator.getAButton()) {
            RobotContainer.vision.setLEDOn(true);
            RobotContainer.shooter.requestShoot(1400, 10);
            RobotContainer.gutNeck.requestShoot(true);
        } else if (RobotContainer.operator.getBButton()) {
            RobotContainer.vision.setLEDOn(true);
            RobotContainer.shooter.requestShoot(1600, 27);
            RobotContainer.gutNeck.requestShoot(true);
        } else {
            RobotContainer.vision.setLEDOn(false);
            RobotContainer.shooter.requestIdle();
            RobotContainer.gutNeck.requestShoot(false);
        }
        if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
            RobotContainer.leftIntake.requestIntake();
            RobotContainer.rightIntake.requestOuttakeRetracted();
            RobotContainer.gutNeck.requestIntakeLeft(true);
        } else if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
            RobotContainer.leftIntake.requestOuttakeRetracted();
            RobotContainer.rightIntake.requestIntake();
            RobotContainer.gutNeck.requestIntakeRight(true);
        } else {
            RobotContainer.gutNeck.requestIntakeLeft(false);
            RobotContainer.gutNeck.requestIntakeRight(false);
            if (RobotContainer.driver.getRawButtonPressed(9)) {
                RobotContainer.leftIntake.requestIdleExtended();
            } else {
                RobotContainer.leftIntake.requestIdleRetracted();
            }
            if (RobotContainer.driver.getRawButton(10)) {
                RobotContainer.rightIntake.requestIdleExtended();
            } else {
                RobotContainer.rightIntake.requestIdleRetracted();
            }
        } 

        if (RobotContainer.operator.getLeftBumper()) {
            RobotContainer.gutNeck.requestSpitLeft(true);
            RobotContainer.leftIntake.requestOuttakeRetracted();
            RobotContainer.rightIntake.requestIdleRetracted();
        } else {
            RobotContainer.gutNeck.requestSpitLeft(false);
        }

        if (RobotContainer.operator.getRightBumper()) {
            RobotContainer.gutNeck.requestSpitRight(true);
            RobotContainer.leftIntake.requestIdleRetracted();
            RobotContainer.rightIntake.requestOuttakeRetracted();
        } else {
            RobotContainer.gutNeck.requestSpitRight(false);
        }
  }

}
