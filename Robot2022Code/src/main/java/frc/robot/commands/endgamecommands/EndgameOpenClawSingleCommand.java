//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameOpenClawSingleCommand</h3>
 * 
 * Opens one of the endgame claws
 */
public class EndgameOpenClawSingleCommand extends CommandBase {

    //-------- DECLARATIONS --------\\

    private final EndgamePistonSubsystem piston;

    //-------- CONSTRUCTOR --------\\
    /**
     * EndgameOpenClawSingleCommand
     * 
     * @param pistonSubsystem the piston to be opened
     */ 
    public EndgameOpenClawSingleCommand(EndgamePistonSubsystem pistonSubsystem) {
        piston = pistonSubsystem;
        addRequirements(pistonSubsystem);
    }
    
    //-------- CLASS METHODS --------\\

    public void initialize() { // Runs once when called
        piston.open();
    }
       
    //Leave false for default command
    public boolean isFinished() { // When true ends command
        return false;
    }
} // End of class EndgameOpenClawCommand
