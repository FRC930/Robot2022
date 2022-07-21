package frc.robot.utilities;

// ----- IMPORTS ----- \\

import org.photonvision.PhotonCamera;

import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>PhotonVisionUtility</h3>
 * 
 * PhotonVisionUtility holds the cameras that we use for our aiming.
 */
public class PhotonVisionUtilityUltimate {
    // ----- CONSTANTS ----- \\
    // The pipeline that you want to select
    private static final int PI_CAMERA_INDEX = 0;

    // ----- VARIABLES ----- \\
    // This is the camera that is used for reflective targets
    private PhotonCamera hubTracking;

    // ----- STATICS ----- \\
    // Singleton flag
    private static PhotonVisionUtilityUltimate instance;

    // ----- CONSTRUCTOR ----- \\
    /**
     * <h3>PhotonVisionUtilityUltimate</h3>
     * 
     * This contstructs the photonvision and should only ever be called once by the
     * getInstance method
     */
    private PhotonVisionUtilityUltimate() {
        // Construction the pipeline
        hubTracking = new PhotonCamera("PiCamera");
        hubTracking.setPipelineIndex(PI_CAMERA_INDEX);
        // Set the banner to show the driver that photon isn't ready yet
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.PHOTON_READY, new ShuffleBoardData<Boolean>(false));
    }

    // ----- SINGLETON GET ----- \\

    /**
     * <h3>getInstance</h3>
     * 
     * This is the accessor method for the singleton. This ensures that there is
     * only ever one instance of the PhotonVisionUtility
     * 
     * @return the instance of PhotonVisionUtility
     */
    public static PhotonVisionUtilityUltimate getInstance() {
        if (instance == null) {
            return (instance = new PhotonVisionUtilityUltimate());
        }
        return instance;
    }

    // ------ METHODS ------ \\

    /**
     * <h3>getHubTrackingCamera</h3>
     * 
     * Returns the PhotonCamera representing the hub aiming camera
     * 
     * @return a reference to the hub camera
     */
    public PhotonCamera getHubTrackingCamera() {
        return hubTracking;
    }
}
