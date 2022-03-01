package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.statemachines.offensive.home.HOME_HOOD;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_NO_CARGO_STOWED;
import frc.robot.statemachines.offensive.idle.IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED;
import frc.robot.sensors.ColorSensor.BallColor;

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
    m_robotContainer.hood.reset();
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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;

    // Boilerplate code to determine how many balls are in the robot and transition to the corresponding state
    if (!homed) {
      CommandScheduler.getInstance().schedule(new HOME_HOOD(m_robotContainer.superstructure));
      homed = true;
    } else {
      if (m_robotContainer.neck.getTopBeamBreak()) {
        CommandScheduler.getInstance().schedule(new IDLE_MODE_ONE_ALLIANCE_CARGO_STOWED(m_robotContainer.superstructure));
      } else {
        CommandScheduler.getInstance().schedule(new IDLE_MODE_NO_CARGO_STOWED(m_robotContainer.superstructure));
      }
    }
    
    // Tracks state changes
    CommandScheduler.getInstance().onCommandInitialize(command -> {
      SmartDashboard.putString("State", command.getName());
    });

    // Cancels the autonomus command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
