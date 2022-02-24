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
 * Manages an interruptible endgame sequence.
 */
public class EndgameManagerCommand extends CommandBase {
    
    //-------- CONSTANTS --------\\

    // Endgame Miscellaneous Constants
    private final double ENDGAME_PISTON_DELAY = 0.5;
    private final double ENDGAME_RELEASE_DELAY = 1;

    //-------- VARIABLES --------\\

    // Map and states for sequence.
    private HashMap<Integer, CommandBase> commands = new HashMap<Integer, CommandBase>();
    private int previousState;
    private int currentState;

     //-------- CONSTRUCTOR --------\\
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
        if (Robot.isReal()) {
            previousState = 1;
            currentState = 1;
        } else {
            previousState = 1;
            currentState = 1;
        }

        //----- ENDGAME COMMAND GROUP INITS -----\\
        // Sets arm to vertical
        commands.put(1,
                new SequentialCommandGroup(
                        new EndgameRotateVerticalCommand(endgameMotorSubsystem, EndgamePosition.ApproachPosition),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 claws
        // Closes #3 claws independently on both sides when sensors trigger
        commands.put(2, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(endgamePistonL3, endgamePistonR3),
                new WaitCommand(ENDGAME_PISTON_DELAY)),
                new ParallelCommandGroup(
                        new EndgameCloseWhenTouching(endgamePistonL3, 4),
                        new EndgameCloseWhenTouching(endgamePistonR3, 4)),
                new WaitCommand(ENDGAME_PISTON_DELAY),
                new EndgameIncrementStateCommand(this)));
        // Opens #1 claws
        // Rotates arm until one #2 sensor triggers, closing all arms and stops motor
        commands.put(3, new SequentialCommandGroup(new ParallelRaceGroup(
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
                        new EndgameRotateVerticalCommand(endgameMotorSubsystem, EndgamePosition.SwingPosition),
                        new EndgameIncrementStateCommand(this)));
        // Opens #3 and #4 claws
        // Gives time for robot swing
        // Closes #4 claws
        // TODO:Change order so it rotates before letting go
        // NOTE:Compesating for weight needs to go reverse
        commands.put(5, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(endgamePistonL3, endgamePistonR3),
                new EndgameOpenClawSingleCommand(endgamePiston4),
                new WaitCommand(ENDGAME_PISTON_DELAY)),
                new WaitCommand(ENDGAME_RELEASE_DELAY),
                new EndgameIncrementStateCommand(this)));
        // Rotates arm until one #4 sensor triggers, closing all arms and stops motor
        commands.put(6, new SequentialCommandGroup(new ParallelRaceGroup(
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
        commands.put(7, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawSingleCommand(endgamePiston2),
                new WaitCommand(ENDGAME_RELEASE_DELAY)), new EndgameIncrementStateCommand(this)));
    }

    /**
     * Increases the state. To be used by EndgameIncrementStateCommand.
     */
    public void nextState() {
        currentState++;
    }

    // Starts the first command
    @Override // Called when the command is initially scheduled.
    public void initialize() {
        CommandScheduler.getInstance().schedule(commands.get(currentState));
    }

    @Override
    public void execute() {
        // Starts next command state if next state is signaled
        if (currentState != previousState && currentState < 8) {
            CommandScheduler.getInstance().cancel(commands.get(previousState));
            CommandScheduler.getInstance().schedule(commands.get(currentState));
            previousState = currentState;
        }
    }

    @Override
    public boolean isFinished() { // when true, ends command
        return currentState == 8;
    }

    @Override
    public void end(boolean interrupted) {
        // Logic to prevent calling null map value
        if (interrupted) {
            CommandScheduler.getInstance().cancel(commands.get(currentState));
        }
        CommandScheduler.getInstance().cancel(commands.get(previousState));
    }

} // End of class EndgameManagerCommand