//----- IMPORTS -----\\

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//----- CLASS -----\\
/**
 * <h3>VisionCameraSubsystem</h3>
 * 
 * This subsystem encapsultes the cameras that we use for vision.
 * Use the {@link frc.robot.subsystems.VisionCameraSubsystem.CameraType
 * CameraType} to specify which camera you wish to access
 */
public class VisionCameraSubsystem extends SubsystemBase {

    //----- CAMERA -----\\

    private PhotonCamera m_visionCamera;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>VisionCameraSubsystem</h3>
     * 
     * Initializes a new subsystem with the passed hardware
     * 
     * @param camera    - the camera to instantiate
     */
    public VisionCameraSubsystem(CameraType camera) {
        if (camera == CameraType.REFLECTIVE_TAPE) {
            m_visionCamera = new PhotonCamera("PiCamera");
            m_visionCamera.setPipelineIndex(0);
        } else {
            m_visionCamera = new PhotonCamera("CargoCamera");
            m_visionCamera.setPipelineIndex(0);
        }
    }

    //----- METHODS -----\\

    /**
     * <h3>getVisionCamera</h3>
     * 
     * @return the PhotonCamera
     */
    public PhotonCamera getVisionCamera() {
        return m_visionCamera;
    }

    //----- ENUMS -----\\

    /**
     * <h3>CameraType</h3>
     * 
     * CameraType represents the three types of cameras
     */
    public static enum CameraType {
        REFLECTIVE_TAPE, BALL_DETECTOR
    }
}
