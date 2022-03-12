package frc.robot.utilities;

import org.photonvision.PhotonCamera;

public class PhotonVisionUtility {
    private PhotonCamera hubTracking = new PhotonCamera("CargoCamera");
    private PhotonCamera ballTracking = new PhotonCamera("PiCamera");

    private static PhotonVisionUtility instance;

    private PhotonVisionUtility() {
        hubTracking.setPipelineIndex(0);
        ballTracking.setPipelineIndex(0);
    }
    
    public static PhotonVisionUtility getInstance() {
        if (instance == null) {
            return new PhotonVisionUtility();
        }
        return instance;
    }

    public PhotonCamera getBallTrackingCamera() {
        return ballTracking;
    }

    public PhotonCamera getHubTrackingCamera() {
        return hubTracking;
    }
}
