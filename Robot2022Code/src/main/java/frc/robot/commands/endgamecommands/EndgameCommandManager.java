package frc.robot.commands.endgamecommands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;

public class EndgameCommandManager extends CommandBase {
    // Endgame Miscellaneous Constants
    private final double ENDGAME_PISTON_DELAY = 0.25;
    private final double ENDGAME_RELEASE_DELAY = 5;

    private HashMap<Integer, CommandBase> commands = new HashMap<Integer, CommandBase>();
    private int currentState;
    
    public EndgameCommandManager(EndgameMotorSubsystem endgameMotorSubsystem,
     EndgamePistonSubsystem left1piston, EndgamePistonSubsystem left2piston, 
     EndgamePistonSubsystem left3piston, EndgamePistonSubsystem left4piston, 
     EndgamePistonSubsystem right1piston, EndgamePistonSubsystem right2piston, 
     EndgamePistonSubsystem right3piston, EndgamePistonSubsystem right4piston) {
        // ----- ENDGAME COMMAND GROUP INITS -----\\
        commands.put(1, new EndgameRotateVerticalCommand(endgameMotorSubsystem));
        // Opens #3 claws
        commands.put(2, new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left3piston, right3piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)));
        // Closes #3 claws independently on both sides when sensors trigger
        commands.put(3, new ParallelCommandGroup(
                new EndgameCloseWhenTouching(left3piston, 4),
                new EndgameCloseWhenTouching(right3piston, 4)));
        // Opens #1 claws
        commands.put(4, new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left1piston, right1piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)));
        // Rotates arm until one #2 sensor triggers, closing all arms and stops motor
        commands.put(5, new ParallelRaceGroup(
                new EndgameArmCommand(endgameMotorSubsystem),
                new EndgameCloseWhenTouching(left1piston, 2),
                new EndgameCloseWhenTouching(right1piston, 2)));
        // Opens #3 and #4 claws
        commands.put(6, new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left3piston, right3piston),
                new EndgameOpenClawPairCommand(left4piston, right4piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)));
        commands.put(7, new WaitCommand(ENDGAME_RELEASE_DELAY));
        // Closes #4 claws
        commands.put(8, new ParallelRaceGroup(
                new EndgameCloseClawPairCommand(left4piston, right4piston),
                new WaitCommand(ENDGAME_PISTON_DELAY)));
        // Rotates arm until one #4 sensor triggers, closing all arms and stops motor
        commands.put(9, new ParallelRaceGroup(
                new EndgameArmCommand(endgameMotorSubsystem),
                new EndgameCloseWhenTouching(left3piston, 4),
                new EndgameCloseWhenTouching(right3piston, 4)));
        // Opens #2 claws-NOTE:Claws closed after delay ends due to default command
        commands.put(10, new ParallelRaceGroup(
                new EndgameOpenClawPairCommand(left2piston, right2piston),
                new WaitCommand(ENDGAME_RELEASE_DELAY)));
        commands.put(11, new EndgameRotateVerticalCommand(endgameMotorSubsystem));
    }
    public void nextState() {
        currentState++;
    }
    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(commands.get(currentState));
    }
    private static enum EndgameStates {
        State1(1), State2(2), State3(3), State4(4), State5(5), State6(6), 
        State7(7), State8(8), State9(9), State10(10), State11(11);

        private int num;

        EndgameStates(int num) {
            this.num = num;
        }
    }
}
