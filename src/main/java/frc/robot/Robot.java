package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Climber;
import frc.robot.sensors.ColorSensor.BallColor;
import static frc.robot.Constants.Hood.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.Climber.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static BallColor allianceColor = BallColor.RED;
  private boolean climbing = false;
  private double lastResetToAbsolute = 0.0;
  private double lastCheckedColorSensorConnected = 0.0;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("Flywheel Set", 0.0);
    SmartDashboard.putNumber("Hood Set", 0.0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double[] d = RobotContainer.gutNeck.colorSensor.getRaw();
    SmartDashboard.putNumber("R", d[0]);
    SmartDashboard.putNumber("G", d[1]);
    SmartDashboard.putNumber("B", d[2]);

    SmartDashboard.putNumber("Latest Vision Pose Heading", getLatestVisonPoseEstimate().getRotation().getDegrees());
    SmartDashboard.putNumber("Rotation Pose", RobotContainer.swerve.getPose().getRotation().getDegrees());
  }

  @Override
  public void disabledInit() {
    lastResetToAbsolute = RobotController.getFPGATime()/1.0E6;
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.vision.setLEDsOn(true);
    if ((RobotController.getFPGATime()/1.0E6) - lastResetToAbsolute > 1.0) {
      RobotContainer.swerve.resetAllToAbsolute();
      lastResetToAbsolute = RobotController.getFPGATime()/1.0E6;
    }
    // if (RobotContainer.operator.getAButton()) {
    //   RobotContainer.climber.commandNeutralMode(true);
    // } else {
    //   RobotContainer.climber.commandNeutralMode(false);
    // }
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
    // RobotContainer.gutNeck.acceptOpposingCargo(false);
    // CHANGFE BNACKLJ AIRF JLAKWEFA:IOWE
    // kljldsjlkdjfslkjlksfkldj
    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;
    
    RobotContainer.swerve.setDriveSlots(0);

    RobotContainer.operator.getAButtonPressed();
    RobotContainer.operator.getYButtonPressed();
    climbing = false;

    lastCheckedColorSensorConnected = RobotController.getFPGATime()/1.0E6;

    // RobotContainer.climber.commandNeutralMode(true);
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
    RobotContainer.vision.setLEDsOn(true);
    RobotContainer.shooter.requestIdle();
  }

  private Pose2d getLatestVisonPoseEstimate() {
    double yawDegrees = RobotContainer.vision.getYaw();
    double targetToCameraMeters = RobotContainer.vision.getCameraToCenterOfHub();
    double visionTimestampSeconds = RobotContainer.vision.getMeasurementTimestamp(); 
    Pose2d visionEstimatedPose = new Pose2d(
        -targetToCameraMeters, 0.0,
        Rotation2d.fromDegrees(-yawDegrees)
    );   
    Pose2d adjustedVisionEstimatePose = new Pose2d(
        visionEstimatedPose.getTranslation().plus(new Translation2d(-CAMERA_TO_CENTER, visionEstimatedPose.getRotation())),
        visionEstimatedPose.getRotation()
    );
    return adjustedVisionEstimatePose;
}

  // Method to handle teleoperated controls
  public void configureTeleopControls() {

    double distance = RobotContainer.vision.getCameraToCenterOfHub();

    SmartDashboard.putBoolean("Valid Shot Distance", distance<=MAX_SHOT_DISTANCE);

    // Driver shooting signals
    if (RobotContainer.driver.getAButton()) {
      RobotContainer.shooter.requestShoot(1400, 10);
      RobotContainer.gutNeck.requestShoot(true);
      RobotContainer.vision.setLEDsOn(true);
    } else if (RobotContainer.driver.getRightStickButton()||RobotContainer.driver.getRightBumper()) {
      RobotContainer.vision.setLEDsOn(true);
      // RobotContainer.shooter.requestShoot(SmartDashboard.getNumber("Flywheel Tuning", 0.0), SmartDashboard.getNumber("Hood Tuning", 8.5));
      if (RobotContainer.swerve.getAtVisionHeadingSetpoint()&&distance<=MAX_SHOT_DISTANCE&&RobotContainer.driver.getLeftStickButton()) {
        RobotContainer.gutNeck.requestShoot(true);
      }
    } else {
      RobotContainer.vision.setLEDsOn(true);
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

    // Driver Max Speeds
    if (RobotContainer.driver.getLeftTriggerAxis() > 0.1 || RobotContainer.driver.getRightTriggerAxis() > 0.1) {
      RobotContainer.swerve.defaultDriveSpeed = 1.5;
    } else {
      RobotContainer.swerve.defaultDriveSpeed = 4.0;
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

    // Handle Temp Climber Controls
    // if (RobotContainer.operator.getRightTriggerAxis() > 0.1) {
    //   RobotContainer.climber.commandPercent(RobotContainer.operator.getRightTriggerAxis() * 0.45);
    // } else if (RobotContainer.operator.getLeftTriggerAxis() > 0.1) {
    //   RobotContainer.climber.commandPercent(-RobotContainer.operator.getLeftTriggerAxis() * 0.45);
    // } else {
    //   RobotContainer.climber.commandPercent(0.0);
    // }

    // if (RobotContainer.operator.getAButtonPressed()) {
    //   RobotContainer.climber.commandSolenoidsForward();
    // }
    // if (RobotContainer.operator.getBButtonPressed()) {
    //   RobotContainer.climber.commandSolenoidsReversed();
    // }

    if (RobotContainer.operator.getYButtonPressed()) {
      if (climbing) {
        climbing = false;
      } else {
        climbing = true;
      }
    }

    if (climbing) {
      RobotContainer.shooter.requestIdle();
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
