package frc.robot.commands.autovisioncommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class HubAimCommand extends SequentialCommandGroup{

    public HubAimCommand(DriveSubsystem dSubsystem){
    new SequentialCommandGroup(new PigeonAimCommand(dSubsystem), new PhotonAimCommand(dSubsystem));
    }
}
