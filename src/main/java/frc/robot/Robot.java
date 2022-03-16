package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.interpolation.InterpolatingTable;
import frc.robot.interpolation.ShotParameter;
import frc.robot.sensors.ColorSensor.BallColor;
import frc.robot.subsystems.climber.ClimbToHighRung;
import frc.robot.subsystems.climber.ClimbToNextRung;
import frc.robot.subsystems.climber.ExtendToMidRung;
import frc.robot.subsystems.climber.LatchToNextRung;
import frc.robot.subsystems.climber.PopOffStaticHooks;
import frc.robot.subsystems.climber.ReadyForNextRung;
import frc.robot.subsystems.climber.TransitioningToNextRung;

import static frc.robot.Constants.Hood.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
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
    SmartDashboard.putNumber("Distance To Target", RobotContainer.vision.getDistance());
    SmartDashboard.putNumber("Pitch", RobotContainer.vision.getPitch());
    SmartDashboard.putNumber("Yaw", RobotContainer.vision.getYaw());
    SmartDashboard.putNumber("Vision Timestamp", RobotContainer.vision.getMeasurementTimestamp());
    RobotContainer.vision.setLEDsOn(true);
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
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {    
    RobotContainer.gutNeck.acceptOpposingCargo(false);
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
            RobotContainer.shooter.requestShoot(1400, 10);
            RobotContainer.gutNeck.requestShoot(true);
        } else if (RobotContainer.operator.getBButton()) {
            // 1600 27
            ShotParameter shot = InterpolatingTable.get(RobotContainer.vision.getDistance());
            RobotContainer.shooter.requestShoot(
              shot.flywheelRPM,
              shot.hoodAngleRadians 
            );
            RobotContainer.gutNeck.requestShoot(true);
        } else {
            RobotContainer.shooter.requestIdle();
            RobotContainer.gutNeck.requestShoot(false);
        }
        if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
            RobotContainer.leftIntake.requestIntake();
            RobotContainer.rightIntake.requestOuttakeRetracted(false);
            RobotContainer.gutNeck.requestIntakeLeft(true);
        } else if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
            RobotContainer.leftIntake.requestOuttakeRetracted(false);
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
            RobotContainer.leftIntake.requestOuttakeRetracted(true);
            RobotContainer.rightIntake.requestIdleRetracted();
        } else {
            RobotContainer.gutNeck.requestSpitLeft(false);
        }

        if (RobotContainer.operator.getRightBumper()) {
            RobotContainer.gutNeck.requestSpitRight(true);
            RobotContainer.leftIntake.requestIdleRetracted();
            RobotContainer.rightIntake.requestOuttakeRetracted(true);
        } else {
            RobotContainer.gutNeck.requestSpitRight(false);
        }

        // if (RobotContainer.operator.getPOV() == 0.0) {
        //   CommandScheduler.getInstance().schedule(new ExtendToMidRung(RobotContainer.climber));
        // } 
        // if (RobotContainer.operator.getPOV() == 180.0) {
        //   CommandScheduler.getInstance().schedule(new ClimbToNextRung(RobotContainer.climber));
        // }
        // if (RobotContainer.operator.getPOV() == 90.0) {
        //   CommandScheduler.getInstance().schedule(new ReadyForNextRung(RobotContainer.climber));
        // }
        // if (RobotContainer.operator.getPOV() == 270.0) {
        //   CommandScheduler.getInstance().schedule(new TransitioningToNextRung(RobotContainer.climber));
        // }
        // if (RobotContainer.operator.getRightStickButton()) {
        //   CommandScheduler.getInstance().schedule(new LatchToNextRung(RobotContainer.climber));
        // }
        // if (RobotContainer.operator.getLeftStickButtonPressed()) {
        //   CommandScheduler.getInstance().schedule(new ClimbToHighRung(RobotContainer.climber));
        // }
        // if (RobotContainer.operator.getXButton()) {
        //   CommandScheduler.getInstance().schedule(new PopOffStaticHooks(RobotContainer.climber));
        // }
        // RobotContainer.climber.commandNeutralMode(RobotContainer.operator.getYButton());

  }

}
