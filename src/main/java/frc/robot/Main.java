package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.simulation.FakeRobot;

public final class Main {
  private Main() {}
  
  public static void main(String... args) {
    if(RobotBase.isReal())
      RobotBase.startRobot(Robot::new);
    else RobotBase.startRobot(FakeRobot::new);
  }
}
