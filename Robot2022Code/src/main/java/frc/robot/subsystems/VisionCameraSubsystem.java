package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3>VisionCameraSubsystem</h3>
 * 
 * This subsystem encapsultes the cameras that we use for vision.
 * Use the {@link frc.robot.subsystems.VisionCameraSubsystem.CameraType
 * CameraType} to specify which camera you wish to access
 */
public class VisionCameraSubsystem extends SubsystemBase {
    private PhotonCamera visionCamera;

    /**
     * <h3>VisionCameraSubsystem</h3>
     * 
     * Initializes a new subsystem with the passed hardware
     * 
     * @param camera the camera to instantiate
     */
    public VisionCameraSubsystem(CameraType camera) {
        if (camera == CameraType.REFLECTIVE_TAPE) {
            visionCamera = new PhotonCamera("PiCamera");
        } else {
            visionCamera = new PhotonCamera("CargoCamera");
        }
    }

    /**
     * <h3>getVisionCamera</h3>
     * 
     * @return the PhotonCamera
     */
    public PhotonCamera getVisionCamera() {
        return visionCamera;
    }

    /**
     * <h3>CameraType</h3>
     * 
     * CameraType represents the three types of cameras
     */
    public static enum CameraType {
        REFLECTIVE_TAPE, BALL_DETECTOR
    }
}
