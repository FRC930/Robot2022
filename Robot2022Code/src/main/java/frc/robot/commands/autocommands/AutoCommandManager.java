package frc.robot.commands.autocommands;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.commands.autocommands.paths.*;
import frc.robot.subsystems.*;

/**
 * <h3>AutonomouseCommandManager</h3>
 * 
 * Manages the autonomous paths by creating an instance of them and putting them
 * into the Shuffleboard.
 */
public class AutoCommandManager {
    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();

    public static enum subNames {
        FlywheelSubsystem("Flywheel"),
        ShooterHoodSubsystem("Shooter Hood"),
        DriveSubsystem("Drive"),
        EndgameMotorSubsystem("Endgame Motor"),
        EndgamePistonSubsystem("Endgame Piston"),
        IntakeMotorSubsystem("Intake Motor"),
        IntakePistonSubsystem("Intake Piston"),
        ShifterSubsystem("Shifter"),
        VisionCameraSubsystem("Vision Camera");

        final String m_name;

        subNames(String name) {
            m_name = name;
        }
    }

    /**
     * Adds a subbsystem to the subystem map
     *
     * @param SubNames
     * @param subsbystem
     */
    public void addSubsystem(subNames SubNames, Subsystem subsystem) {
        subsystemMap.put(SubNames.toString(), subsystem);
    }

    /**
     * Creates instances of each autonomous path command
     */
    public void initCommands() {

        TarmacTaxi defaultAutoPathCommand = new TarmacTaxi(
                (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()));

        TaxiOneBall defaultShoot = new TaxiOneBall(
                (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString())
                //(CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
                );

        TaxiTwoBall TwoBallAuto = new TaxiTwoBall(
                (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
                (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
                (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString())
                //(CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
                );

        TerminalPickup TerminalPickup = new TerminalPickup(
                (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
                (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
                (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString())
                //(CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
                );

        DefensiveThreeBall DefensiveThreeBall = new DefensiveThreeBall(
                (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
                (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
                (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString())
                //(CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
                );

        DefensiveHangarCleanup DefensiveHangarCleanup = new DefensiveHangarCleanup(
            (DriveSubsystem) subsystemMap.get(subNames.DriveSubsystem.toString()),
            (IntakePistonSubsystem) subsystemMap.get(subNames.IntakePistonSubsystem.toString()),
            (IntakeMotorSubsystem) subsystemMap.get(subNames.IntakeMotorSubsystem.toString())
            //(CatapultSubsystem) subsystemMap.get(subNames.CatapultSubsystem.toString())
            );

        // Adding auto paths to the Shuffleboard
        ShuffleboardUtility.getInstance().setDefaultAutonOptions("(None)", null);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultAutoPathCommand", defaultAutoPathCommand);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultShootingCommand", defaultShoot);

        ShuffleboardUtility.getInstance().addAutonOptions("TwoBallAuto", TwoBallAuto);
        // ShuffleboardUtility.getInstance().addAutonOptions("ShootMoveShoot",
        // ShootMoveShoot);
        ShuffleboardUtility.getInstance().addAutonOptions("TerminalPickup", TerminalPickup);
        ShuffleboardUtility.getInstance().addAutonOptions("DefensiveThreeBall", DefensiveThreeBall);
        ShuffleboardUtility.getInstance().addAutonOptions("DefensiveHangarCleanup", DefensiveHangarCleanup);
    }

    /**
     *
     * Gets the autonomous path that is selected in the Shuffleboard
     *
     * @return The selected autonomous commandl
     */
    public Command getAutonomousCommand() {
        return ShuffleboardUtility.getInstance().getSelectedAutonPath();
    }

}
