package frc.robot.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {

    public ColorSensorV3 sensor;
    private ColorMatch match = new ColorMatch();
    private final Color redBallTarget, blueBallTarget, noneTarget;
    private final I2C.Port port;


    public ColorSensor(I2C.Port port, Color redBallTarget, Color blueBallTarget, Color noneTarget) {
        sensor = new ColorSensorV3(port);
        sensor.configureColorSensor(ColorSensorResolution.kColorSensorRes16bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain1x);
        match.addColorMatch(redBallTarget);
        match.addColorMatch(blueBallTarget);
        match.addColorMatch(noneTarget);
        this.redBallTarget = redBallTarget;
        this.blueBallTarget = blueBallTarget;
        this.noneTarget = noneTarget;
        this.port = port;
    }

    public BallColor get() {
        Color detectedColor = sensor.getColor();
        ColorMatchResult result = match.matchClosestColor(detectedColor);
        if (result.color == redBallTarget) {
            return BallColor.RED;
        } else if (result.color == blueBallTarget) {
            return BallColor.BLUE;
        } else if (result.color == noneTarget) {
            return BallColor.NONE;
        } else {
            return BallColor.NONE;
        }
    }

    public boolean hasReset() {
        return sensor.getBlue() == 0.0 && sensor.getGreen() == 0.0 && sensor.getRed() == 0.0;
    }

    public double[] getRaw() {
        Color color = sensor.getColor();
        return new double[] {
            color.red,
            color.green,
            color.blue
        };
    }
    
    public enum BallColor {
        RED, BLUE, NONE
    }
    
}
