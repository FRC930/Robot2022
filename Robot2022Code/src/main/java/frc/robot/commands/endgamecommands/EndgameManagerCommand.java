package frc.robot.commands.endgamecommands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;

/**
 * <h3>EndgameManagerCommand</h3>
 * 
 * Manages an interruptible endgame sequence.
 */
public class EndgameManagerCommand extends CommandBase {
    // Endgame Miscellaneous Constants
    private final double ENDGAME_PISTON_DELAY = 0.5;
    private final double ENDGAME_RELEASE_DELAY = 1;

    // Map and states for sequence.
    private HashMap<Integer, CommandBase> commands = new HashMap<Integer, CommandBase>();
    private int previousState;
    private int currentState;

    /**
     * EndgameManagerCommand
     * <p>
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
            previousState = 2;
            currentState = 2;
        } else {
            previousState = 2;
            currentState = 2;
        }

        // ----- ENDGAME COMMAND GROUP INITS -----\\
        // Sets arm to vertical
        commands.put(1, new SequentialCommandGroup(new EndgameRotateVerticalCommand(endgameMotorSubsystem),
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
        // Opens #3 and #4 claws
        // Gives time for robot swing
        // Closes #4 claws
        // TODO:Change order so it rotates before letting go
        // NOTE:Compesating for weight needs to go reverse
        commands.put(4, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(endgamePistonL3, endgamePistonR3),
                new EndgameOpenClawSingleCommand(endgamePiston4),
                new WaitCommand(ENDGAME_PISTON_DELAY)),
                new WaitCommand(ENDGAME_RELEASE_DELAY),
                new EndgameIncrementStateCommand(this)));
        // Rotates arm until one #4 sensor triggers, closing all arms and stops motor
        commands.put(5, new SequentialCommandGroup(new ParallelRaceGroup(
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
        commands.put(6, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawSingleCommand(endgamePiston2),
                new WaitCommand(ENDGAME_RELEASE_DELAY)), new EndgameIncrementStateCommand(this)));
        // Sets arm to vertical
        commands.put(7, new SequentialCommandGroup(new EndgameRotateVerticalCommand(endgameMotorSubsystem),
                new EndgameIncrementStateCommand(this)));
    }

    /**
     * Increases the state. To be used by EndgameIncrementStateCommand.
     */
    public void nextState() {
        currentState++;
    }

    // Starts the first command
    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(commands.get(currentState));
    }

    @Override
    public void execute() {
        // Starts next command state if next state is signaled
        if (currentState != previousState && currentState < 7) {
            CommandScheduler.getInstance().cancel(commands.get(previousState));
            CommandScheduler.getInstance().schedule(commands.get(currentState));
            previousState = currentState;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == 7;
    }

    @Override
    public void end(boolean interrupted) {
        // Logic to prevent calling null map value
        if (interrupted) {
            CommandScheduler.getInstance().cancel(commands.get(currentState));
        }
        CommandScheduler.getInstance().cancel(commands.get(previousState));
    }
}
