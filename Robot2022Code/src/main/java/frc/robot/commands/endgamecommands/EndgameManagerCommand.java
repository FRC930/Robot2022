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
    private final double ENDGAME_PISTON_DELAY = 0.25;
    private final double ENDGAME_RELEASE_DELAY = 5;

    //Map and states for sequence.
    private HashMap<Integer, CommandBase> commands = new HashMap<Integer, CommandBase>();
    private int previousState;
    private int currentState;

    /**
     * EndgameManagerCommand<p>
     * Creates new manager for endgame.
     * @param endgameMotorSubsystem
     * @param left1piston
     * @param left2piston
     * @param left3piston
     * @param left4piston
     * @param right1piston
     * @param right2piston
     * @param right3piston
     * @param right4piston
     */
    public EndgameManagerCommand(EndgameMotorSubsystem endgameMotorSubsystem,
            EndgamePistonSubsystem left1piston, EndgamePistonSubsystem left2piston,
            EndgamePistonSubsystem left3piston, EndgamePistonSubsystem left4piston,
            EndgamePistonSubsystem right1piston, EndgamePistonSubsystem right2piston,
            EndgamePistonSubsystem right3piston, EndgamePistonSubsystem right4piston) {
        if (Robot.isReal()) {
            previousState = 1;
            currentState = 1;
        } else {
            previousState = 2;
            currentState = 2;
        }

        // ----- ENDGAME COMMAND GROUP INITS -----\\
        //Sets arm to vertical
        commands.put(1, new SequentialCommandGroup(new EndgameRotateVerticalCommand(endgameMotorSubsystem),
                new EndgameIncrementStateCommand(this)));
        // Opens #3 claws
        commands.put(2, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left3piston, right3piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)), new EndgameIncrementStateCommand(this)));
        // Closes #3 claws independently on both sides when sensors trigger
        commands.put(3, new SequentialCommandGroup(new ParallelCommandGroup(
                new EndgameCloseWhenTouching(left3piston, 4),
                new EndgameCloseWhenTouching(right3piston, 4)), new EndgameIncrementStateCommand(this)));
        // Opens #1 claws
        commands.put(4, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left1piston, right1piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)), new EndgameIncrementStateCommand(this)));
        // Rotates arm until one #2 sensor triggers, closing all arms and stops motor
        commands.put(5, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameArmCommand(endgameMotorSubsystem),
                new EndgameCloseWhenTouching(left1piston, 2),
                new EndgameCloseWhenTouching(right1piston, 2)), new EndgameIncrementStateCommand(this)));
        // Opens #3 and #4 claws
        commands.put(6, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left3piston, right3piston),
                new EndgameOpenClawPairCommand(left4piston, right4piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)), new EndgameIncrementStateCommand(this)));
        //Gives time for robot swing
        commands.put(7,
                new SequentialCommandGroup(new WaitCommand(ENDGAME_RELEASE_DELAY),
                        new EndgameIncrementStateCommand(this)));
        // Closes #4 claws
        commands.put(8, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameCloseClawPairCommand(left4piston, right4piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)), new EndgameIncrementStateCommand(this)));
        // Rotates arm until one #4 sensor triggers, closing all arms and stops motor
        commands.put(9, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameArmCommand(endgameMotorSubsystem),
                new EndgameCloseWhenTouching(left3piston, 4),
                new EndgameCloseWhenTouching(right3piston, 4)), new EndgameIncrementStateCommand(this)));
        // Opens #2 claws-NOTE:Claws closed after delay ends due to default command
        commands.put(10, new SequentialCommandGroup(new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left2piston, right2piston),
                new WaitCommand(ENDGAME_RELEASE_DELAY)), new EndgameIncrementStateCommand(this)));
        //Sets arm to vertical
        commands.put(11, new SequentialCommandGroup(new EndgameRotateVerticalCommand(endgameMotorSubsystem),
                new EndgameIncrementStateCommand(this)));
    }

    /**
     * Increases the state. To be used by EndgameIncrementStateCommand.
     */
    public void nextState() {
        currentState++;
    }

    //Starts the first command
    @Override
    public void initialize(){
        CommandScheduler.getInstance().schedule(commands.get(currentState));
    }

    @Override
    public void execute() {
        //Starts next command state if next state is signaled
        if (currentState != previousState && currentState < 12) {
            CommandScheduler.getInstance().cancel(commands.get(previousState));
            CommandScheduler.getInstance().schedule(commands.get(currentState));
            previousState = currentState;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == 12;
    }

    @Override
    public void end(boolean interrupted) {
        //Logic to prevent calling null map value
        if(interrupted){
            CommandScheduler.getInstance().cancel(commands.get(currentState));
        }
        CommandScheduler.getInstance().cancel(commands.get(previousState));
    }
}
