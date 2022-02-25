package frc.robot.commands.autocommands;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.commands.autocommands.paths.*;
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

        CommandBase defaultAutoPathCommand = new DefaultAutoPathCommand(
            (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString())
        );

        CommandBase defaultShoot = new DefaultShoot(
            (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()), 
            (CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
        );

        CommandBase CompPath1 = new TwoBallAuto(
            (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
            (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
            (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString()),
            (VisionCameraSubsystem) subsystemMap.get(subNames.VisionCameraSubsystem.toString()),
            (CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
        );

        CommandBase ShootMoveShoot = new ShootMoveShoot(
            (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
            (CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString()),
            (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
            (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString())
        );
        
        CommandBase TerminalPickup = new TerminalPickup(
            (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
            (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
            (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString()),
            (VisionCameraSubsystem) subsystemMap.get(subNames.VisionCameraSubsystem.toString()),
            (CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
        );

        ShuffleboardUtility.getInstance().setDefaultAutonOptions("(None)", null);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultAutoPathCommand", defaultAutoPathCommand);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultShootingCommand", defaultShoot);

        ShuffleboardUtility.getInstance().addAutonOptions("CompPath1", CompPath1);
        ShuffleboardUtility.getInstance().addAutonOptions("ShootMoveShoot", ShootMoveShoot);
        ShuffleboardUtility.getInstance().addAutonOptions("TerminalPickup", TerminalPickup);
    }

    public Command getAutonomousCommand(){
        return ShuffleboardUtility.getInstance().getSelectedAutonPath();
    }
 
}
