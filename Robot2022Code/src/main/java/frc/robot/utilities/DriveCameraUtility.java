package frc.robot.utilities;

/**
 * <h3>DriveCameraUtility</h3>
 * 
 * DriveCameraUtility stores the state of the camera for drive subsystem
 * 
 * @author Alexander Taylor
 * @author Jack LaFreniere
 * @since 29 January 2022
 * @version 1.0
 */
public class DriveCameraUtility {
    // The only instance of the class
    private static DriveCameraUtility cameraUtility;

    // The current state of the drive camera
    private CameraStates cameraState;
    private BallColor ballColor;

    /**
     * <h3>DriveCameraUtility</h3>
     * 
     * Initializes a new drive camera utility with the default of reflective tape
     */
    private DriveCameraUtility() {
        cameraState = CameraStates.REFLECTIVE_TAPE;
    }

    /**
     * <h3>getCameraState</h3>
     * 
     * Gets the current camera state
     * 
     * @return the current camera state
     */
    public CameraStates getCameraState() {
        return cameraState;
    }

    /**
     * <h3>setCameraState</h3>
     * 
     * Sets the current camera state
     * 
     * @param state the current camera state
     */
    public void setCameraState(CameraStates state) {
        cameraState = state;
    }

    public BallColor getBallColor() {
        return ballColor;
    }

    public void setBallColor(BallColor color) {
        ballColor = color;
    }

    /**
     * <h3>getInstace</h3>
     * 
     * Returns the instance of the singleton
     * 
     * @return a {@link frc.robot.utilities.DriveCameraUtility DriveCameraUtility}
     *         from the singleton
     */
    public static DriveCameraUtility getInstance() {
        if (cameraUtility == null) {
            cameraUtility = new DriveCameraUtility();
        }
        return cameraUtility;
    }

    /**
     * <h3>CameraStates</h3>
     * 
     * CameraStates represents the state of the drive camera
     * 
     * @author Alexander Taylor
     * @author Jack LaFreniere
     * @since 29 January 2022
     * @version 1.0
     */
    public static enum CameraStates {
        REFLECTIVE_TAPE, BALL
    }

    public static enum BallColor {
        BLUE, RED
    }
}