package frc.robot.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import frc.robot.commons.BreadUtil;
import frc.robot.sensors.PicoColorSensor.RawColor;
import edu.wpi.first.wpilibj.util.Color;
import static frc.robot.Constants.Gut.*;

public class ColorSensor {

    public PicoColorSensor sensor;
    ColorMatch match = new ColorMatch();
    Color redBallTarget, blueBallTarget, noneTarget;
    private double connectionFaultTriggered = 0.0;
    private double timeoutFaultTriggered = 0.0;


    public ColorSensor() {
        sensor = new PicoColorSensor();    
        match.addColorMatch(RED_TARGET);
        match.addColorMatch(BLUE_TARGET);
        match.addColorMatch(NONE_TARGET);
        this.redBallTarget = RED_TARGET;
        this.blueBallTarget = BLUE_TARGET;
        this.noneTarget = NONE_TARGET;  
    }

    public BallColor get() {
        Color detectedColor = getColor();
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

    public int[] getRGB() {
        RawColor color = sensor.getRawColor0();
        return new int[] {
            color.red,
            color.blue,
            color.green
        };
    }

    public Color getColor() {
        int[] rgb = getRGB();
        double mag = rgb[0] + rgb[1] + rgb[2];
        return new Color((double) rgb[0] / mag, (double) rgb[1] / mag, (double) rgb[2] / mag);
    }

    public float[] getHSB() {
        int[] color = getRGB();
        return RGBtoHSB(color[0], color[1], color[2]);
    }

    public void triggerConnectionFault() {
        connectionFaultTriggered = 1.0;
    }

    public boolean isConnected() {
        return sensor.isSensor0Connected();
    }

    public double getConnectionFaultTriggered() {
        return connectionFaultTriggered;
    }

    public void triggerTimeoutFault() {
        timeoutFaultTriggered = 1.0;
    }

    public boolean hasNotTimeout() {
        return BreadUtil.getFPGATimeSeconds() - sensor.getLastReadTimestampSeconds() < 0.5;
    }

    public double getTimeoutFaultTriggered() {
        return timeoutFaultTriggered;
    }

    public static float[] RGBtoHSB(int r, int g, int b) {
        float hue, saturation, brightness;
        float[] hsbvals = new float[3];
        
        int cmax = (r > g) ? r : g;
        if (b > cmax) cmax = b;
        int cmin = (r < g) ? r : g;
        if (b < cmin) cmin = b;
        
        brightness = ((float) cmax) / 255.0f;
        if (cmax != 0)
            saturation = ((float) (cmax - cmin)) / ((float) cmax);
        else
            saturation = 0;
        if (saturation == 0)
            hue = 0;
        else {
            float redc = ((float) (cmax - r)) / ((float) (cmax - cmin));
            float greenc = ((float) (cmax - g)) / ((float) (cmax - cmin));
            float bluec = ((float) (cmax - b)) / ((float) (cmax - cmin));
            if (r == cmax)
            hue = bluec - greenc;
            else if (g == cmax)
                hue = 2.0f + redc - bluec;
                else
            hue = 4.0f + greenc - redc;
            hue = hue / 6.0f;
            if (hue < 0)
            hue = hue + 1.0f;
        }
        hsbvals[0] = hue;
        hsbvals[1] = saturation;
        hsbvals[2] = brightness;
        return hsbvals;
    }
    
    public enum BallColor {
        RED, BLUE, NONE
    }
    
}
