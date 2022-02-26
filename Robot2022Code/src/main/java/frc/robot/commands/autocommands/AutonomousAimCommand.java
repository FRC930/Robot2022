package frc.robot.commands.autocommands;

import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.commands.autovisioncommands.HubAimingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;

public class AutonomousAimCommand extends HubAimingCommand {
    public AutonomousAimCommand(VisionCameraSubsystem cameraSubsystem, DriveSubsystem dSubsystem) {
        super(cameraSubsystem, dSubsystem);

        addRequirements(cameraSubsystem, dSubsystem);
    }

    @Override
    public boolean isFinished() {
        boolean isAimed = false;
        double xDegreeOffset = 1;
        //
        PhotonPipelineResult result = this.reflectiveTapeCamera.getVisionCamera().getLatestResult();
        //
        if(result.hasTargets()){
            xDegreeOffset = -result.getBestTarget().getYaw();
        }
        //
        if(xDegreeOffset > -1 && xDegreeOffset < 1){
            isAimed = true;
        }

        return isAimed;
    }
}
