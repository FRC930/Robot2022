package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * <h3>EndgameIncrementStateCommand</h3>
 * 
 * Increases state in EndgameManagerCommand
 */
public class EndgameIncrementStateCommand extends CommandBase {

    private final EndgameManagerCommand managerCommand;

    /**
     * EndgameIncrementStateCommand
     * 
     * @param mCommand EndgameManagerCommand object to use
     */
    public EndgameIncrementStateCommand(EndgameManagerCommand mCommand) {
        managerCommand = mCommand;
    }

    @Override
    public void initialize() {
        managerCommand.nextState();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}