package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.vision.Vision;
import frc.robot.autonomus.AutonomusSelector;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.statemachines.GutNeck;
import frc.robot.subsystems.statemachines.Intake;
import frc.robot.subsystems.statemachines.Shooter;
import frc.robot.subsystems.swerve.DefaultDriveController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.VisionFollowerController;
import static frc.robot.Constants.DualIntake.*;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class RobotContainer {

  public static Swerve swerve = new Swerve();
  public static Shooter shooter = new Shooter();
  public static Intake leftIntake = new Intake(LEFT_INTAKE_ID, TalonFXInvertType.Clockwise, LEFT_INTAKE_PISTON_CHANNELS[0], LEFT_INTAKE_PISTON_CHANNELS[1], 0);
  public static Intake rightIntake = new Intake(RIGHT_INTAKE_ID, TalonFXInvertType.CounterClockwise, RIGHT_INTAKE_PISTON_CHANNELS[0], RIGHT_INTAKE_PISTON_CHANNELS[1], 0);
  public static GutNeck gutNeck = new GutNeck();
  public static Vision vision = new Vision();
  public static ClimberIOTalonFX climberIO = new ClimberIOTalonFX();
  public static Climber climber = new Climber(climberIO);
  public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static XboxController driver = new XboxController(0);
  public static XboxController operator = new XboxController(1);
  public static AutonomusSelector autoSelector = new AutonomusSelector(swerve, shooter, leftIntake, rightIntake, gutNeck);
  public static AutonomusSelector autonomusSelector = new AutonomusSelector(swerve, shooter, leftIntake, rightIntake, gutNeck);

  public RobotContainer() {
    swerve.setDefaultCommand(new DefaultDriveController(swerve));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kStart.value).whenPressed(
      new InstantCommand(() -> swerve.reset(new Pose2d()), swerve)
    );

    new JoystickButton(driver, Button.kRightStick.value).whileHeld(
      new VisionFollowerController(swerve, false)
    );

    new JoystickButton(driver, Button.kRightBumper.value).whileHeld(
      new VisionFollowerController(swerve, false)
    );

  }

  public Command getAutonomousCommand() {
    return autonomusSelector.get();
  }
}