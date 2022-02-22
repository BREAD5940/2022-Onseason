package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class BeamBreak {

    private final AnalogInput input;

    public BeamBreak(int channel) {
        this.input = new AnalogInput(channel);
    }

    public double getRaw() {
        return input.getVoltage() / RobotController.getVoltage5V();
    }

    public boolean get() {
        return getRaw() > 0.5 ? false : true;
    }
    
}