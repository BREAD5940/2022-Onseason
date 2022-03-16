package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    
    // Variables
    public final PhotonCamera camera = new PhotonCamera("BreadCam");
    private double yaw;
    private double pitch;
    private double distance;
    private double timestampSeconds;

    public Vision() {
        NetworkTableInstance.getDefault()
            .getEntry("/photonvision/" + "BreadCam" + "/latencyMillis")
            .addListener(event -> {
                PhotonPipelineResult result = camera.getLatestResult();
                timestampSeconds = RobotController.getFPGATime()/1.0E6 - Units.millisecondsToSeconds(result.getLatencyMillis());
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                yaw = bestTarget.getYaw();
                pitch = bestTarget.getPitch();
                distance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS, 
                    TARGET_HEIGHT_METERS, 
                    CAMERA_PITCH_RADIANS, 
                    Units.degreesToRadians(pitch)
                );
            }, EntryListenerFlags.kUpdate);

    }
    
    // Method to get the yaw
    public double getYaw() {
        return -yaw;
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
        camera.setLED(set ? VisionLEDMode.kOn : VisionLEDMode.kOff);
    }

    
}
