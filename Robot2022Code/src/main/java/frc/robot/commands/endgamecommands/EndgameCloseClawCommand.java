package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;

/**
 * <h3> EndgameCloseClawCommand</h3>
 * 
 * Closes one of the endgame claws
 */
public class EndgameCloseClawCommand extends CommandBase{

    private final EndgamePistonSubsystem piston;
    /**
     * EndgameCloseClawCommand
     * 
     * @param pistonSubsystem the piston you want to close
     */
    public EndgameCloseClawCommand(EndgamePistonSubsystem pistonSubsystem) {
        piston = pistonSubsystem;
    }
    
    public void initialize() { // runs once when called
        piston.closed();
    }
    
    public boolean isFinished() { // when true, ends command
       return true;
    }
}
