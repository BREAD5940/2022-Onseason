package frc.robot.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {

    public final ColorSensorV3 sensor;
    private final ColorMatch match = new ColorMatch();
    private final Color redBallTarget, blueBallTarget, noTarget;


    public ColorSensor(I2C.Port port, Color redBallTarget, Color blueBallTarget, Color noTarget) {
        sensor = new ColorSensorV3(port);
        match.addColorMatch(redBallTarget);
        match.addColorMatch(blueBallTarget);
        match.addColorMatch(noTarget);
        this.redBallTarget = redBallTarget;
        this.blueBallTarget = blueBallTarget;
        this.noTarget = noTarget;
    }

    public BallColor get() {
        Color detectedColor = sensor.getColor();
        ColorMatchResult result = match.matchClosestColor(detectedColor);
        if (result.color == redBallTarget) {
            return BallColor.RED;
        } else if (result.color == blueBallTarget) {
            return BallColor.BLUE;
        } else if (result.color == noTarget) {
            return BallColor.NONE;
        } else {
            return BallColor.NONE;
        }
    }
    
    public enum BallColor {
        RED, BLUE, NONE
    }
    
}
