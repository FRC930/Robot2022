/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Robot;
import frc.robot.commands.endgamecommands.EndgameRotateVerticalCommand.EndgamePosition;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameManagerCommand</h3>
 * 
 * Manages an interruptible endgame sequence state machine.
 */
public class EndgameManagerCommand extends CommandBase {

    // -------- CONSTANTS --------\\

    // TODO: TUNE VALUES FOR FASTER CLIMB
    // Time delay for claw commands to take effect
    private final double ENDGAME_PISTON_DELAY = 0.4;
    // Time delay for letting go of Mid while hangning from High
    private final double ENDGAME_RELEASE_DELAY = 0.75;

    // -------- VARIABLES --------\\
    // Map of states for the sequence
    private HashMap<Integer, CommandBase> commands = new HashMap<Integer, CommandBase>();
    // State flag for currently used state command
    private int currentState;
    // State flag for updatable state
    private int newState;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameManagerCommand</h3>
     * 
     * Creates new manager for endgame.
     * 
     * @param endgameMotorSubsystem
     * @param endgamePiston1
     * @param endgamePiston2
     * @param endgamePistonL3
     * @param endgamePistonR3
     * @param endgamePiston4
     */
    public EndgameManagerCommand(EndgameMotorSubsystem endgameMotorSubsystem, EndgamePistonSubsystem endgamePiston1,
            EndgamePistonSubsystem endgamePiston2, EndgamePistonSubsystem endgamePistonL3,
            EndgamePistonSubsystem endgamePistonR3, EndgamePistonSubsystem endgamePiston4) {
        // Starts at the beginning of the map sequence
        currentState = 1;
        newState = 1;

        // ----- ENDGAME COMMAND GROUP INITS -----\\
        // Sets arm to vertical to approach
        commands.put(1,
                new SequentialCommandGroup(
                        new EndgameRotateVerticalCommand(endgameMotorSubsystem,
                                EndgamePosition.ApproachPosition),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 claws
        // Closes #3 claws independently on both sides when sensors trigger
        commands.put(2,
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new EndgameOpenClawPairCommand(endgamePistonL3,
                                        endgamePistonR3),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
                        new ParallelCommandGroup(
                                new EndgameCloseWhenTouching(endgamePistonL3, 4),
                                new EndgameCloseWhenTouching(endgamePistonR3, 4)),
                        new WaitCommand(ENDGAME_PISTON_DELAY),
                        new EndgameIncrementStateCommand(this)));
        // Opens #1 claws
        // Rotates arm until one #2 sensor triggers, closing all arms and stops motor
        commands.put(3,
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new EndgameOpenClawSingleCommand(endgamePiston1),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
                        new ParallelRaceGroup(
                                new EndgameArmCommand(endgameMotorSubsystem),
                                new EndgameCloseWhenTouching(endgamePiston1, 2)),
                        new WaitCommand(ENDGAME_PISTON_DELAY),
                        new EndgameIncrementStateCommand(this)));
        // Sets arm to vertical
        commands.put(4,
                new SequentialCommandGroup(
                        new EndgameRotateVerticalCommand(endgameMotorSubsystem, 
                            EndgamePosition.SwingPosition),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 and #4 claws
        // Gives time for robot swing
        // Closes #4 claws
        commands.put(5,
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new EndgameOpenClawPairCommand(endgamePistonL3, endgamePistonR3),
                                new EndgameOpenClawSingleCommand(endgamePiston4),
                                new WaitCommand(ENDGAME_RELEASE_DELAY)),
                        new EndgameIncrementStateCommand(this)));
        // Rotates arm until one #4 sensor triggers, closing all arms and stops motor
        commands.put(6,
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new EndgameCloseClawSingleCommand(endgamePiston4),
                                new EndgameOpenClawPairCommand(endgamePistonL3, endgamePistonR3),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
                        new ParallelRaceGroup(
                                new EndgameArmCommand(endgameMotorSubsystem),
                                new EndgameCloseWhenTouching(endgamePistonL3, 4),
                                new EndgameCloseWhenTouching(endgamePistonR3, 4)),
                        new WaitCommand(ENDGAME_PISTON_DELAY),
                        new EndgameIncrementStateCommand(this)));
        // Opens #2 claws-NOTE:Claws closed after delay ends due to default command
        commands.put(7,
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new EndgameOpenClawSingleCommand(endgamePiston2),
                                new WaitCommand(ENDGAME_RELEASE_DELAY)),
                        new EndgameIncrementStateCommand(this)));
    }

    /**
     * Increases the state. To be used by EndgameIncrementStateCommand.
     */
    public void nextState() {
        newState++;
    }

    // Starts the first command
    @Override // Called when the command is initially scheduled.
    public void initialize() {
        CommandScheduler.getInstance().schedule(commands.get(currentState));
    }

    @Override
    public void execute() {
        // Starts next command state if next state is signaled
        if (newState > currentState && newState <= commands.size()) {
            CommandScheduler.getInstance().cancel(commands.get(currentState));
            CommandScheduler.getInstance().schedule(commands.get(newState));
            currentState = newState;
        }
    }

    @Override
    public boolean isFinished() { // when true, ends command
        return newState == commands.size() + 1;
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(commands.get(currentState));
    }

} // End of class EndgameManagerCommand