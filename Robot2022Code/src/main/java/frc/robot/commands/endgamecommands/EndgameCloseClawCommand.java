package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
//-------- COMMAND CLASS --------\\
/**
 * <h3> EndgameCloseClawCommand</h3>
 * 
 * Closes one of the endgame claws
 */
public class EndgameCloseClawCommand extends CommandBase{

    //-------- DECLARATIONS --------\\

    private final EndgamePistonSubsystem piston;

    //-------- CONSTRUCTOR --------\\
    /**
     * EndgameCloseClawCommand
     * 
     * @param pistonSubsystem the piston you want to close
     */
    public EndgameCloseClawCommand(EndgamePistonSubsystem pistonSubsystem) {
        piston = pistonSubsystem;
        addRequirements(pistonSubsystem);
    }

    //-------- CLASS METHODS  --------\\
    
    public void initialize() { // runs once when called
        piston.closed();
    }
    
    public boolean isFinished() { // when true, ends command
       return true;
    }
} // End of class EndgameCloseClawCommand
