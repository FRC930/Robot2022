//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameOpenClawCommand</h3>
 * 
 * Opens one of the endgame claws
 */
public class EndgameOpenClawCommand extends CommandBase {

    //-------- DECLARATIONS --------\\

    private final EndgamePistonSubsystem piston;

    //-------- CONSTRUCTOR --------\\
    /**
     * EndgameOpenClawCommand
     * 
     * @param pistonSubsystem the piston to be opened
     */ 
    public EndgameOpenClawCommand(EndgamePistonSubsystem pistonSubsystem) {
        piston = pistonSubsystem;
        addRequirements(pistonSubsystem);
    }
    
    //-------- CLASS METHODS --------\\

    public void initialize() { // Runs once when called
        piston.open();
    }
       
    public boolean isFinished() { // When true ends command
        return true;
    }
} // End of class EndgameOpenClawCommand
