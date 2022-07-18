/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Robot;
import frc.robot.commands.endgamecommands.EndgameCloseWhenTouching.EndgameSensorPairs;
import frc.robot.commands.endgamecommands.EndgameRotateArmCommand.EndgamePosition;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameManagerCommand</h3>
 * 
 * Manages an interruptible endgame sequence state machine.
 */
public class EndgameManagerCommand extends CommandBase {

    // -------- CONSTANTS --------\\
    // Time delay for claw state changes to take effect
    private final double ENDGAME_PISTON_DELAY = 0.5;
    // Sensor debouncer time
    private final double SENSOR_DELAY_TIME = 0.1;
    // Arm power to apply during climb/sensor search
    private final double ENDGAME_MOTOR_POWER = 1.0;

    // -------- VARIABLES --------\\
    // Map(key value pairs) of states for the sequence
    private HashMap<Integer, CommandBase> commands = new HashMap<Integer, CommandBase>();
    // State flag for currently used state command
    private int currentState;
    // State flag for updatable state
    private int newState;
    // Compressor object to disable
    private Compressor m_compressor;

    private EndgamePistonSubsystem endgamePiston1;
    private EndgamePistonSubsystem endgamePiston2;
    private EndgamePistonSubsystem endgamePiston3;
    private EndgamePistonSubsystem endgamePiston4;

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
     * @param indexerMotorSubsystem the indexer subsystem to stop
     * @param compressor            the robot compressor to stop
     */
    public EndgameManagerCommand(EndgameMotorSubsystem endgameMotorSubsystem, EndgamePistonSubsystem endgamePiston1,
            EndgamePistonSubsystem endgamePiston2, EndgamePistonSubsystem endgamePiston3,
            EndgamePistonSubsystem endgamePiston4, IndexerMotorSubsystem indexerMotorSubsystem,
            Compressor compressor) {

        this.endgamePiston1 = endgamePiston1;
        this.endgamePiston2 = endgamePiston2;
        this.endgamePiston3 = endgamePiston3;
        this.endgamePiston4 = endgamePiston4;

        // ----- ENDGAME COMMAND GROUP INITS -----\\

        // Sets arm to vertical to approach
        commands.put(1,
                new SequentialCommandGroup(
                        // Sets arm to vertical to approach
                        new EndgameRotateArmCommand(endgameMotorSubsystem,
                                EndgamePosition.ApproachPosition),
                        new EndgameIncrementStateCommand(this)));

        // Opens #3 claws
        // Closes #3 claws when both #4 sensors trigger
        commands.put(2,
                new SequentialCommandGroup(
                        // Opens #3 claws
                        new EndgameOpenClawCommand(endgamePiston3),
                        // Closes #3 claws when both #4 sensors trigger
                        new EndgameCloseWhenTouching(endgamePiston3,
                                EndgameSensorPairs.SensorPair4, SENSOR_DELAY_TIME),
                        // Wait to close claws
                        new WaitCommand(ENDGAME_PISTON_DELAY),
                        new EndgameIncrementStateCommand(this)));

        // Opens #1 claws
        // Rotates arm until both #2 sensors trigger
        // Then closes the #2 claws and stops the motor
        commands.put(3,
                new SequentialCommandGroup(
                        // Opens #1 claws
                        new EndgameOpenClawCommand(endgamePiston1),
                        // Rotates arm until both #2 sensors trigger
                        // Then closes the #2 claws and stops the motor
                        new ParallelRaceGroup(
                                new EndgameArmCommand(endgameMotorSubsystem,
                                        ENDGAME_MOTOR_POWER),
                                new SequentialCommandGroup(
                                        new EndgameCloseWhenTouching(
                                                endgamePiston1,
                                                EndgameSensorPairs.SensorPair2,
                                                SENSOR_DELAY_TIME),
                                        new WaitCommand(0.2))),
                        new WaitCommand(1.0),
                        new EndgameIncrementStateCommand(this)));
        
        // resets the position of the endagame arms
        commands.put(4,  new SequentialCommandGroup(
                new EndgameRotateArmCommand(endgameMotorSubsystem, EndgamePosition.BalancePosition),
                new EndgameIncrementStateCommand(this)
        ));

        // Opens #3 and #4 claws, waits extra before letting go
        commands.put(5,
                new SequentialCommandGroup(
                        // Opens #3 and #4 claws, waits extra before letting go
                        new ParallelCommandGroup(
                                new EndgameOpenClawCommand(endgamePiston3),
                                new EndgameOpenClawCommand(endgamePiston4)),
                        // this wait since next step is to
                        // step for detecting the bar
                        new WaitCommand(3.25),
                        new EndgameIncrementStateCommand(this)));

        // Opens #3 claw and closes #4
        // Rotates arm until both #4 sensor triggers
        // Then closes all arms and stops motor
        commands.put(6,
                new SequentialCommandGroup(
                        // Opens #3 claw and closes #4
                        new ParallelRaceGroup(
                                new EndgameCloseClawCommand(endgamePiston4),
                                new EndgameOpenClawCommand(endgamePiston3)),
                        // Rotates arm until both #4 sensor triggers
                        // Then closes #3 claws and stops motor
                        new ParallelRaceGroup(
                                new EndgameArmCommand(endgameMotorSubsystem,
                                        ENDGAME_MOTOR_POWER),
                                new SequentialCommandGroup(
                                        new EndgameCloseWhenTouching(
                                                endgamePiston3,
                                                EndgameSensorPairs.SensorPair4,
                                                SENSOR_DELAY_TIME),
                                        new WaitCommand(0.2))),
                        new WaitCommand(1), // SHORTEN THIS
                        new EndgameIncrementStateCommand(this)));

        // Opens #2 claws
        // NOTE:Claws close automatically after the final stage ends due to default
        // commands
        commands.put(7,
                new SequentialCommandGroup(
                        // Opens #2 claws
                        new EndgameOpenClawCommand(endgamePiston2),
                        new EndgameArmCommand(endgameMotorSubsystem, ENDGAME_MOTOR_POWER)
                                .withTimeout(0.5),
                        new EndgameIncrementStateCommand(this)));

        // Start state machine values
        resetState();

        // Assign the compressor and indexer to stop upon initialization
        m_compressor = compressor;
        addRequirements(indexerMotorSubsystem);
    }

    /**
     * Increases the state. To be used by EndgameIncrementStateCommand.
     */
    public void nextState() {
        newState++;
    }

    /**
     * Resets the state machine back to zero.
     */
    public void resetState() {
        // Starts at the beginning of the map sequence
        // Simulation cannot use arm encoder, need to start at step 2
        if (Robot.isReal()) {
            currentState = 1;
            newState = 1;
        } else {
            currentState = 2;
            newState = 2;
        }
    }

    // Starts the first command
    @Override // Called when the command is initially scheduled.
    public void initialize() {
        m_compressor.disable();
        CommandScheduler.getInstance().schedule(commands.get(currentState));
    }

    @Override
    public void execute() {
        // Ends current command and starts next command if newState is updated
        if (newState > currentState && newState <= commands.size()) {
            CommandScheduler.getInstance().cancel(commands.get(currentState));
            CommandScheduler.getInstance().schedule(commands.get(newState));
            currentState = newState;
        }
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, ShuffleboardKeys.ENDGAME_STATE, new ShuffleBoardData<Integer>(currentState));
        System.out.println("1: " + endgamePiston1.isOpen() + " 2: " + endgamePiston2.isOpen() 
        + " 3: " + endgamePiston3.isOpen() + " 4: " + endgamePiston4.isOpen());
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
    public void end(boolean interrupted) { // Interrupted is true when button is released
        // Ends the command currently running
        m_compressor.enableAnalog(100, 115);
        CommandScheduler.getInstance().cancel(commands.get(currentState));
    }

} // End of class EndgameManagerCommand