package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sensors.ColorSensor.BallColor;
import frc.robot.subsystems.statemachines.GutNeck.GutNeckStates;
import static frc.robot.Constants.Hood.*;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.Flywheel.*;

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
    SmartDashboard.putNumber("Flywheel Calibration", FLYWHEEL_CALIBRATION);
    SmartDashboard.putNumber("F-Mounting-Adjustment", 0.0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Color d = RobotContainer.gutNeck.colorSensor.getColor();
    float[] f = RobotContainer.gutNeck.colorSensor.getHSB();
    SmartDashboard.putNumber("R", d.red);
    SmartDashboard.putNumber("G", d.green);
    SmartDashboard.putNumber("B", d.blue);
    SmartDashboard.putString("Ball Color", RobotContainer.gutNeck.colorSensor.get().name());

    SmartDashboard.putNumber("Hue", f[0]);
    SmartDashboard.putNumber("Saturation", f[1]);
    SmartDashboard.putNumber("Value", f[2]);

    // System.out.printf("Detected: %s, Value: %.3f\n", RobotContainer.gutNeck.colorSensor.get().name(), f[2]);

    // if (Math.abs(d[0]) < 1.0E-6 && Math.abs(d[1]) < 1.0E-6 && Math.abs(d[2]) < 1.0E-6) {
    //   System.out.println("REV Color Sensor Has Died\nTrying to Re-initialize\n");
    //   RobotContainer.gutNeck.colorSensor.intialize();
    // } TODO

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
    if (RobotContainer.operator.getXButton()) {
      RobotContainer.climber.commandNeutralMode(true);
    } else {
      RobotContainer.climber.commandNeutralMode(false);
    }
  }

  @Override
  public void autonomousInit() {
    // Set alliance color
    allianceColor = DriverStation.getAlliance() == Alliance.Red ? BallColor.RED : BallColor.BLUE;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    RobotContainer.gutNeck.requestReset();
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
      RobotContainer.swerve.defaultDriveSpeed = 3.0;
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

    // Operator climber controls
    if (RobotContainer.operator.getAButtonPressed()) {
      RobotContainer.climber.requestNextState();
    } 
    if (RobotContainer.operator.getBButtonPressed()) {
      RobotContainer.climber.requestPreviousState();
    }

    // Driver vibrations
    if (RobotContainer.gutNeck.getSystemState() == GutNeckStates.IDLE_TWO_CARGO) {
      RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0.35);
    } else {
      RobotContainer.driver.setRumble(RumbleType.kRightRumble, 0.0);
    }

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
