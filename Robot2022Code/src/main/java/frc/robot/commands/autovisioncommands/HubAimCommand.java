package frc.robot.commands.autovisioncommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class HubAimCommand extends SequentialCommandGroup{

    public HubAimCommand(DriveSubsystem driveSubsystem){
        new SequentialCommandGroup(
            new PigeonAimCommand(driveSubsystem), 
            new PhotonAimCommand(driveSubsystem)
        );
    }

    public HubAimCommand(
        DriveSubsystem driveSubsystem, 
        XboxController driverController, 
        XboxController coDriverController
    ) {
        new SequentialCommandGroup(
            new PigeonAimCommand(driveSubsystem), 
            new PhotonAimCommand(driveSubsystem, driverController, coDriverController)
        );
    }
}