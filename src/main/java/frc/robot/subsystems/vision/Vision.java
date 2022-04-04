package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;

import java.util.TreeMap;

import static frc.robot.Constants.Autonomus.*;

public class Vision extends SubsystemBase {
    
    // Variables
    public final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    TreeMap<Double, Double> timeInterpolatingBuffer = new TreeMap<Double, Double>();
    private double yaw;
    private double pitch;
    private double originalPitch;
    private double distance;
    private double timestampSeconds;

    public Vision() {
        // NetworkTableInstance.getDefault()
        //     .getEntry("/photonvision/" + "BreadCam" + "/latencyMillis")
        //     .addListener(event -> {
        //         PhotonPipelineResult result = camera.getLatestResult();
        //         if (!result.hasTargets())  {
        //             return;
        //         }
        //         PhotonTrackedTarget bestTarget = result.getBestTarget();
        //         if (bestTarget != null) {
        //             timestampSeconds = RobotController.getFPGATime()/1.0E6 - Units.millisecondsToSeconds(result.getLatencyMillis()) - Units.millisecondsToSeconds(60.0);
        //             yaw = bestTarget.getYaw();
        //             pitch = bestTarget.getPitch();
        //             distance = PhotonUtils.calculateDistanceToTargetMeters(
        //                 CAMERA_HEIGHT_METERS, 
        //                 TARGET_HEIGHT_METERS, 
        //                 CAMERA_PITCH_RADIANS, 
        //                 Units.degreesToRadians(pitch)
        //             );
        //         }
        //     }, EntryListenerFlags.kUpdate);
        limelightTable
            .getEntry("tl")
            .addListener(event -> {
                if (limelightTable.getEntry("tv").getDouble(0.0) == 0) {
                    return;
                }
                double pixelOffset = limelightTable.getEntry("tshort").getDouble(0.0)/2.0;
                timestampSeconds = RobotController.getFPGATime()/1.0E6 - Units.millisecondsToSeconds(limelightTable.getEntry("tl").getDouble(0.0)) - Units.millisecondsToSeconds(11.0);
                yaw = limelightTable.getEntry("tx").getDouble(0.0);
                double centerCrosshairPitch = limelightTable.getEntry("ty").getDouble(0.0);
                originalPitch = centerCrosshairPitch;
                double centerCrosshairY = Math.tan(Units.degreesToRadians(centerCrosshairPitch)) * CAMERA_BASIS_PIXELS;
                double topCrosshairY = centerCrosshairY + pixelOffset;
                pitch = Units.radiansToDegrees(Math.atan(topCrosshairY/CAMERA_BASIS_PIXELS));
                distance = setDistance(calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS, 
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(pitch)
                ));
                SmartDashboard.putNumber("Pixel offset", pixelOffset);
                SmartDashboard.putNumber("Center Crosshair Y", centerCrosshairY);
                SmartDashboard.putNumber("topCrossHairY", topCrosshairY);
            }, EntryListenerFlags.kUpdate);
    }
    
    // Method to get the yaw
    public double getYaw() {
        return -yaw-LIMELIGHT_MOUNTING_ANGLE_ERROR;
    }
        
    // Method to get the pitch
    public double getPitch() {
        return pitch;
    }

    // Method to get the distance 
    public double getDistance() {
        return distance;
    }

    // Method to get the latency in milliseconds
    public double getMeasurementTimestamp() {
        return timestampSeconds;
    }


    // Method to turn set the limelight LEDs
    public void setLEDsOn(boolean set) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(set ? 0.0 : 1.0);
    }

    // Method to calculate the distance to the target
    private double calculateDistanceToTargetMeters(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians) {
        return (targetHeightMeters - cameraHeightMeters)
                / Math.tan(cameraPitchRadians + targetPitchRadians);
    }

    // Private method to add a distance to the time interpolating buffer
    private double setDistance(double currentDistance) {
        timeInterpolatingBuffer.put(RobotController.getFPGATime()/1.0E6, currentDistance);
        if (RobotController.getFPGATime()/1.0E6 - timeInterpolatingBuffer.firstEntry().getKey() > 0.5) {
            timeInterpolatingBuffer.pollFirstEntry();
        }
        double sum = 0.0;
        int tot = timeInterpolatingBuffer.size();
        for (double rpm : timeInterpolatingBuffer.values()) {
            sum += rpm;
        }
        return (double) sum / tot;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance To Target", getDistance());
        SmartDashboard.putNumber("Center Pitch", originalPitch);
        SmartDashboard.putNumber("Adjusted Pitch", getPitch());
        SmartDashboard.putNumber("Yaw", getYaw());
        SmartDashboard.putNumber("Vision Timestamp", getMeasurementTimestamp());
    }
    
}
