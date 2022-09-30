package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.Drive.*;

import java.util.TreeMap;

public class Vision extends SubsystemBase {
    
    // Variables
    public final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    TreeMap<Double, Double> timeInterpolatingBuffer = new TreeMap<Double, Double>();
    private double yaw;
    private double pitch;
    private double originalPitch;
    private double distance;
    private double timestampSeconds;
    private double hasTarget;
    private double mountingAngle = MOUNTING_PITCH;
    private double mountingAdjustment = 0.0;

    public Vision() {
        limelightTable
            .getEntry("tl")
            .addListener(event -> {
                if (limelightTable.getEntry("tv").getDouble(0.0) == 0) {
                    return;
                }
                double pixelOffset = limelightTable.getEntry("tshort").getDouble(0.0)/2.0;
                timestampSeconds = BreadUtil.getFPGATimeSeconds() - Units.millisecondsToSeconds(limelightTable.getEntry("tl").getDouble(0.0)) - Units.millisecondsToSeconds(11.0);
                yaw = -limelightTable.getEntry("tx").getDouble(0.0);
                hasTarget = limelightTable.getEntry("tv").getDouble(0.0);
                double centerCrosshairPitch = limelightTable.getEntry("ty").getDouble(0.0);
                originalPitch = centerCrosshairPitch;
                double centerCrosshairY = Math.tan(Units.degreesToRadians(centerCrosshairPitch)) * CAMERA_BASIS_PIXELS;
                double topCrosshairY = centerCrosshairY + pixelOffset;
                pitch = Units.radiansToDegrees(Math.atan(topCrosshairY/CAMERA_BASIS_PIXELS));
                distance = setDistance(getCameraToTarget().getNorm());
                SmartDashboard.putNumber("Pixel offset", pixelOffset);
                SmartDashboard.putNumber("Center Crosshair Y", centerCrosshairY);
                SmartDashboard.putNumber("topCrossHairY", topCrosshairY);
            }, EntryListenerFlags.kUpdate);
    }
    
    // Method to get the yaw
    public double getYaw() {
        Translation2d cameraToTarget = getCameraToTarget();
        return new Rotation2d(cameraToTarget.getX(), cameraToTarget.getY()).getDegrees();
    }
        
    // Method to get the pitch
    public double getPitch() {
        return pitch;
    }

    // Method to get the distance 
    public double getCameraToCenterOfHub() {
        return distance + UPPER_HUB_RADIUS;
    }

    // Method to get the latency in milliseconds
    public double getMeasurementTimestamp() {
        return timestampSeconds;
    }


    // Method to turn set the limelight LEDs
    public void setLEDsOn(boolean set) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(set ? 0.0 : 1.0);
    }

    // Method to check if vision is working
    public boolean working() {
        return hasTarget == 1.0;
    }

    // Private method to add a distance to the time interpolating buffer
    private double setDistance(double currentDistance) {
        timeInterpolatingBuffer.put(RobotController.getFPGATime()/1.0E6, currentDistance);
        if (RobotController.getFPGATime()/1.0E6 - timeInterpolatingBuffer.firstEntry().getKey() > 0.25) {
            timeInterpolatingBuffer.pollFirstEntry();
        }
        double sum = 0.0;
        int tot = timeInterpolatingBuffer.size();
        for (double d : timeInterpolatingBuffer.values()) {
            sum += d;
        }
        return (double) sum / tot;
    }

    // Private method that returns the distance to the target
    public Translation2d getCameraToTarget() {
        // Define the vector
        double x = 1.0 * Math.tan(Units.degreesToRadians(yaw));
        double y = 1.0 * Math.tan(Units.degreesToRadians(pitch));
        double z = 1.0;
        double norm = Math.sqrt(x*x+y*y+z*z);
        x /= norm;
        y /= norm;
        z /= norm;

        // Rotate the vector by the camera pitch
        double xPrime = x;
        Translation2d yzPrime = new Translation2d(y, z).rotateBy(new Rotation2d(-mountingAngle));
        double yPrime = yzPrime.getX();
        double zPrime = yzPrime.getY();

        // Solve for the intersection
        double angleToGoalRadians = Math.asin(yPrime);
        double diffHeight = TARGET_HEIGHT_METERS - LENS_HEIGHT_METERS;
        double distance = diffHeight/Math.tan(angleToGoalRadians);

        // Returns the distance to the target (in meters)
        return new Translation2d(distance, Rotation2d.fromDegrees(yaw-Units.radiansToDegrees(MOUNTING_YAW)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance To Center Of Hub", getCameraToCenterOfHub());
        SmartDashboard.putNumber("Center Pitch", originalPitch);
        SmartDashboard.putNumber("Adjusted Pitch", getPitch());
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("Vision Timestamp", getMeasurementTimestamp());
        mountingAdjustment = Units.degreesToRadians(SmartDashboard.getNumber("F-Mounting-Adjustment", 0.0));
        mountingAngle = MOUNTING_PITCH + mountingAdjustment;
    }
    
}
