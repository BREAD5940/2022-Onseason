package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.interpolation.InterpolatingTable;
import frc.robot.interpolation.ShotParameter;
import frc.robot.sensors.ColorSensor.BallColor;
import frc.robot.subsystems.climber.Climber.ClimberActions;
import static frc.robot.Constants.Flywheel.*;

import static frc.robot.Constants.Hood.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static BallColor allianceColor = BallColor.RED;
  private ClimberActions nextClimberAction = ClimberActions.GO_TO_MID_RUNG_HEIGHT;
  private boolean climbing = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("Flywheel Callibration", FLYWHEEL_CALIBRATION);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Distance To Target", RobotContainer.vision.getDistance());
    SmartDashboard.putNumber("Pitch", RobotContainer.vision.getPitch());
    SmartDashboard.putNumber("Yaw", RobotContainer.vision.getYaw());
    SmartDashboard.putNumber("Vision Timestamp", RobotContainer.vision.getMeasurementTimestamp());
    double[] d = RobotContainer.gutNeck.colorSensor.getRaw();
    SmartDashboard.putNumber("R (sensor)", d[0]);
    SmartDashboard.putNumber("G (sensor)", d[1]);
    SmartDashboard.putNumber("B (sensor)", d[2]);
    SmartDashboard.putString("Next Climber Action", nextClimberAction.name());
    SmartDashboard.putNumber("Compressor Pressure", RobotContainer.compressor.getPressure());
    // System.out.println(RobotContainer.gutNeck.colorSensor.get().name());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    RobotContainer.vision.setLEDsOn(false);
  }

  @Override
  public void autonomousInit() {
    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    RobotContainer.swerve.setDriveSlots(1);
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.vision.setLEDsOn(true);
  }

  @Override
  public void teleopInit() {    
    RobotContainer.gutNeck.acceptOpposingCargo(false);
    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;
    
    RobotContainer.swerve.setDriveSlots(0);
  }

  @Override
  public void teleopPeriodic() {
    RobotContainer.vision.setLEDsOn(true);
    configureTeleopControls();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    RobotContainer.vision.setLEDsOn(false);
    RobotContainer.shooter.requestIdle();
  }

  // Method to handle teleoperated controls
  public void configureTeleopControls() {

    // Driver shooting signals
    if (RobotContainer.driver.getAButton()) {
      RobotContainer.shooter.requestShoot(1400, 10);
      RobotContainer.gutNeck.requestShoot(true);
    } else if (RobotContainer.driver.getRightStickButton()) {
      ShotParameter shot = InterpolatingTable.get(RobotContainer.vision.getDistance());
      RobotContainer.shooter.requestShoot(shot.flywheelRPM, shot.hoodAngleRadians);
      if (RobotContainer.swerve.getAtVisionHeadingSetpoint()) {
        RobotContainer.gutNeck.requestShoot(true);
      }
    } else {
      RobotContainer.shooter.requestShoot(1000.0, HOOD_IDLE_POS);
      RobotContainer.gutNeck.requestShoot(false);
    }

    // Driver intaking signals
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
      RobotContainer.leftIntake.requestIdleRetracted();
      RobotContainer.rightIntake.requestIdleRetracted();
    } 

    // Operator spit left button
    if (RobotContainer.operator.getLeftBumper()) {
      RobotContainer.gutNeck.requestSpitLeft(true);
      RobotContainer.leftIntake.requestOuttakeRetracted(true);
      RobotContainer.rightIntake.requestIdleRetracted();
    } else {
      RobotContainer.gutNeck.requestSpitLeft(false);
    }

    // Operator spit right button
    if (RobotContainer.operator.getRightBumper()) {
      RobotContainer.gutNeck.requestSpitRight(true);
      RobotContainer.leftIntake.requestIdleRetracted();
      RobotContainer.rightIntake.requestOuttakeRetracted(true);
    } else {
      RobotContainer.gutNeck.requestSpitRight(false);
    }

    // Overrides the intake signals from the driver
    if (RobotContainer.operator.getLeftStickButton()) {
      RobotContainer.leftIntake.requestOuttakeExtended(false);
    } 
    if (RobotContainer.operator.getRightStickButton()) {
      RobotContainer.rightIntake.requestOuttakeExtended(false);
    } 

    // Overrides the gutneck intake signals from the driver
    if (RobotContainer.operator.getLeftTriggerAxis() > 0.1) {
      RobotContainer.gutNeck.requestIntakeLeft(true);
      RobotContainer.gutNeck.requestIntakeRight(false);
      RobotContainer.shooter.requestIdle();
      RobotContainer.gutNeck.requestShoot(false);
    }
    if (RobotContainer.operator.getRightTriggerAxis() > 0.1) {
      RobotContainer.gutNeck.requestIntakeRight(true);
      RobotContainer.gutNeck.requestIntakeLeft(false);
      RobotContainer.shooter.requestIdle();
      RobotContainer.gutNeck.requestShoot(false);
    }

    // Buttons for the operator to control what cargo is accepted
    if (RobotContainer.operator.getPOV() == 0.0) {
      RobotContainer.gutNeck.acceptOpposingCargo(false);
    } else if (RobotContainer.operator.getPOV() == 90.0) {
      RobotContainer.gutNeck.acceptOpposingCargo(true);
    } 

    // "Auto" Climber Buttons
    if (RobotContainer.operator.getAButtonPressed()) {
      if (nextClimberAction != ClimberActions.DONE) 
        CommandScheduler.getInstance().schedule(RobotContainer.climber.getCommandFromAction(nextClimberAction));
      nextClimberAction = RobotContainer.climber.getNextClimberAction(nextClimberAction);
    }
    if (RobotContainer.operator.getBButtonPressed()) {
      nextClimberAction = RobotContainer.climber.getPreviousClimberAction(nextClimberAction);
      CommandScheduler.getInstance().schedule(RobotContainer.climber.getCommandFromAction(nextClimberAction));
    }

    if (RobotContainer.operator.getYButton()) {
      if (climbing) {
        climbing = false;
      } else {
        climbing = true;
      }
    }

    if (climbing) {
      RobotContainer.shooter.requestIdle();
      RobotContainer.vision.setLEDsOn(false);
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
