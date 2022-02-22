package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
    private volatile PhotonTrackedTarget bestTarget = null;

    public Vision() {
        NetworkTableInstance.getDefault()
            .getEntry("/photonvision/" + CAMERA_NAME + "/latencyMillis")
            .addListener(event -> {
                PhotonPipelineResult result = camera.getLatestResult();
                bestTarget = result.getBestTarget();
            }, EntryListenerFlags.kUpdate);
    }

    public double getDistanceToTarget() {
        return PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS, 
            TARGET_HEIGHT_METERS, 
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(bestTarget.getPitch())
        );
    }


    
}
