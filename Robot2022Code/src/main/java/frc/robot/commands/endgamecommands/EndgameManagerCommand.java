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
    // Time delay for claw commands to take effect
    private final double ENDGAME_PISTON_DELAY = 0.5;

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
     * @param endgameMotorSubsystem the endgame motor
     * @param endgamePiston1        the pair of left and right 1 pistons
     * @param endgamePiston2        the pair of left and right 2 pistons
     * @param endgamePiston3        the pair of left and right 3 pistons
     * @param endgamePiston4        the pair of left and right 4 pistons
     */
    public EndgameManagerCommand(EndgameMotorSubsystem endgameMotorSubsystem, EndgamePistonSubsystem endgamePiston1,
            EndgamePistonSubsystem endgamePiston2, EndgamePistonSubsystem endgamePiston3,
            EndgamePistonSubsystem endgamePiston4) {
        // Starts at the beginning of the map sequence
        // Simulation cannot rotate arm, need to start at step 2
        if (Robot.isReal()) {
            currentState = 1;
            newState = 1;
        } else {
            currentState = 2;
            newState = 2;
        }

        // ----- ENDGAME COMMAND GROUP INITS -----\\
        // Sets arm to vertical to approach
        commands.put(1,
                new SequentialCommandGroup(
                        // Sets arm to vertical to approach
                        new EndgameRotateVerticalCommand(endgameMotorSubsystem,
                                EndgamePosition.ApproachPosition),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 claws
        // Closes #3 claws when both #4 sensors trigger
        commands.put(2,
                new SequentialCommandGroup(
                        // Opens #3 claws
                        new ParallelRaceGroup(
                                new EndgameOpenClawCommand(endgamePiston3),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
                        // Closes #3 claws when both #4 sensors trigger
                        new EndgameCloseWhenTouching(endgamePiston3, 4),
                        new WaitCommand(ENDGAME_PISTON_DELAY),
                        new EndgameIncrementStateCommand(this)));
        // Opens #1 claws
        // Rotates arm until both #2 sensors trigger
        // Then closes the #2 claws and stops the motor
        commands.put(3,
                new SequentialCommandGroup(
                        // Opens #1 claws
                        new ParallelRaceGroup(
                                new EndgameOpenClawCommand(endgamePiston1),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
                        // Rotates arm until both #2 sensors trigger
                        // Then closes the #2 claws and stops the motor
                        new ParallelRaceGroup(
                                new EndgameArmCommand(endgameMotorSubsystem),
                                new EndgameCloseWhenTouching(endgamePiston1, 2)),
                        new WaitCommand(ENDGAME_PISTON_DELAY),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 and #4 claws, waits extra before letting go
        commands.put(4,
                new SequentialCommandGroup(
                        // Opens #3 and #4 claws, waits extra before letting go
                        new ParallelRaceGroup(
                                new EndgameOpenClawCommand(endgamePiston3),
                                new EndgameOpenClawCommand(endgamePiston4),
                                new WaitCommand(ENDGAME_PISTON_DELAY * 2)),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 claw and closes #4
        // Rotates arm until both #4 sensor triggers
        // Then closes all arms and stops motor
        commands.put(5,
                new SequentialCommandGroup(
                        // Opens #3 claw and closes #4
                        new ParallelRaceGroup(
                                new EndgameCloseClawCommand(endgamePiston4),
                                new EndgameOpenClawCommand(endgamePiston3),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
                        // Rotates arm until both #4 sensor triggers
                        // Then closes #3 claws and stops motor
                        new ParallelRaceGroup(
                                new EndgameArmCommand(endgameMotorSubsystem),
                                new SequentialCommandGroup(
                                        new EndgameCloseWhenTouching(endgamePiston3, 4),
                                        new WaitCommand(ENDGAME_PISTON_DELAY * 2)
                                )
                        ),
                        new EndgameIncrementStateCommand(this))
                );
        // Opens #2 claws
        // NOTE:Claws close automatically after the final stage ends due to default
        // commands
        commands.put(6,
                new SequentialCommandGroup(
                        // Opens #2 claws
                        new ParallelRaceGroup(
                                new EndgameOpenClawCommand(endgamePiston2),
                                new WaitCommand(ENDGAME_PISTON_DELAY)),
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

    /**
     * <h3>isFinished</h3>
     * Determines if the manager is finished
     * 
     * @return if new state is beyond the limits of the map
     */
    @Override
    public boolean isFinished() { // when true, ends command
        return newState > commands.size();
    }

    @Override
    public void end(boolean interrupted) { // Interrupted when button is released
        CommandScheduler.getInstance().cancel(commands.get(currentState));
    }

} // End of class EndgameManagerCommand