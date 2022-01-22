package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCameraSubsystem extends SubsystemBase {
    private PhotonCamera visionCamera;

    public VisionCameraSubsystem(String cameraName) {
        visionCamera = new PhotonCamera(cameraName);
    }

    public PhotonCamera getVisionCamera() {
        return visionCamera;
    }
}
