package frc.robot.commands.autocommands;

import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.commands.autovisioncommands.HubAimingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;

/**
 * <h3>AutonomousAimCommand</h3>
 * 
 * Aims the robot while in autonomous
 */
public class AutonomousAimCommand extends HubAimingCommand {
    public AutonomousAimCommand(VisionCameraSubsystem cameraSubsystem, DriveSubsystem dSubsystem) {
        super(cameraSubsystem, dSubsystem);
    }

    @Override
    /**
     * Aims the robot while in autonomous
     * 
     * @return Whether or not the robot has aiming at the right target
     */
    public boolean isFinished() {

        PhotonPipelineResult result = super.reflectiveTapeCamera.getVisionCamera().getLatestResult();

        if (result.hasTargets()) {
            double xDegreeOffset = -result.getBestTarget().getYaw();

            if (xDegreeOffset > -1 && xDegreeOffset < 1) {
                return true;
            }
        }

        return false;
    }
}
