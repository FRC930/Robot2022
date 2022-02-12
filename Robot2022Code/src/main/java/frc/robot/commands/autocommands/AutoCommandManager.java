package frc.robot.commands.autocommands;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.commands.autocommands.paths.AutoTest1;
import frc.robot.commands.autocommands.paths.AutoTest2;
import frc.robot.commands.autocommands.paths.AutoTestFull;
import frc.robot.commands.autocommands.paths.BottomBackShootCommand;
import frc.robot.commands.autocommands.paths.BottomBackSideShootCommand;
import frc.robot.commands.autocommands.paths.CompPath1;
import frc.robot.commands.autocommands.paths.DefaultAutoPathCommand;
import frc.robot.subsystems.*;

public class AutoCommandManager {
    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();

    public static enum subNames{
        CatapultSensorSubsystem("Catapult Sensor"),
        CatapultSubsystem("Catapult"),
        DriveSubsystem("Drive"),
        EndgameMotorSubsystem("Endgame Motor"),
        EndgamePistonSubsystem("Endgame Piston"),
        EndgameSensorSubsystem("Endgame Sensor"),
        IntakeMotorSubsystem("Intake Motor"),
        IntakePistonSubsystem("Intake Piston"),
        ShifterSubsystem("Shifter"),
        VisionCameraSubsystem("Vision Camera");

        final String m_name;

        subNames(String name) {
            m_name = name;
        }
    }
    
    public void addSubsystem(subNames SubNames, Subsystem subsystem){
        subsystemMap.put(SubNames.toString(), subsystem);
    }

    public void initCommands(){
        CommandBase defaultAutoPathCommand = new DefaultAutoPathCommand((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));
        CommandBase bottomBackSideShootCommand = new BottomBackSideShootCommand((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));
        CommandBase bottomBackShootCommand = new BottomBackShootCommand((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));
        CommandBase AutoTest1 = new AutoTest1((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));
        CommandBase AutoTest2 = new AutoTest2((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));
        CommandBase AutoTestFull = new AutoTestFull((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));
        CommandBase CompPath1 = new CompPath1((DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));

        ShuffleboardUtility.getInstance().setDefaultAutonOptions("Default (None)", null);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultAutoPathCommand", defaultAutoPathCommand);
        ShuffleboardUtility.getInstance().addAutonOptions("bottomBackShootCommand", bottomBackShootCommand);
        ShuffleboardUtility.getInstance().addAutonOptions("bottomBackSideShootCommand", bottomBackSideShootCommand);
        ShuffleboardUtility.getInstance().addAutonOptions("AutoTest1", AutoTest1);
        ShuffleboardUtility.getInstance().addAutonOptions("AutoTest2", AutoTest2);
        ShuffleboardUtility.getInstance().addAutonOptions("AutoTestFull", AutoTestFull);
        ShuffleboardUtility.getInstance().addAutonOptions("CompPath1", CompPath1);
    }

    public Command getAutonomousCommand(){
        return ShuffleboardUtility.getInstance().getSelectedAutonPath();
    }
 
}
