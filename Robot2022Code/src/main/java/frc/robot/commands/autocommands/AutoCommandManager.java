package frc.robot.commands.autocommands;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.commands.autocommands.paths.BottomBackShootCommand;
import frc.robot.commands.autocommands.paths.BottomBackSideShootCommand;
import frc.robot.commands.autocommands.paths.DefaultAutoPathCommand;
import frc.robot.subsystems.*;

public class AutoCommandManager {
    HashMap<String, CommandBase> commandMap = new HashMap<>();
    HashMap<String, Subsystem> subsystemMap = new HashMap<>();

    public static enum subNames{
        CatapultSensorSubsystem,
        CatapultSubsystem,
        DriveSubsystem,
        EndgameMotorSubsystem,
        EndgamePistonSubsystem,
        EndgameSensorSubsystem,
        IntakeMotorSubsystem,
        IntakePistonSubsystem,
        ShifterSubsystem,
        VisionCameraSubsystem
    }

    public AutoCommandManager(DriveSubsystem driveSubsystem){
        CommandBase defaultAutoPathCommand = new DefaultAutoPathCommand(driveSubsystem);
        CommandBase bottomBackSideShootCommand = new BottomBackSideShootCommand(driveSubsystem);
        CommandBase bottomBackShootCommand = new BottomBackShootCommand(driveSubsystem);

        commandMap.put("defaultAutoPathCommand", defaultAutoPathCommand);
        commandMap.put("bottomBackSideShootCommand", bottomBackSideShootCommand);
        commandMap.put("bottomBackShootCommand", bottomBackShootCommand);

        ShuffleboardUtility.getInstance().setDefaultAutonOptions("Default (None)", null);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultAutoPathCommand", commandMap.get("defaultAutoPathCommand"));
        ShuffleboardUtility.getInstance().addAutonOptions("bottomBackShootCommand", commandMap.get("bottomBackSideShootCommand"));
        ShuffleboardUtility.getInstance().addAutonOptions("bottomBackSideShootCommand", commandMap.get("bottomBackShootCommand"));
    }
    /*
    public void addSubsystem(){
        subsystemMap.put("key", subNames.CatapultSensorSubsystem);
    }
*/
    public Command getAutonomousCommand(){
        return ShuffleboardUtility.getInstance().getSelectedAutonPath();
    }
 
}
