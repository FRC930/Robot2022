package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
//-------- COMMAND CLASS --------\\
/**
 * <h3> EndgameCloseClawSingleCommand</h3>
 * 
 * Closes one of the endgame claws
 */
public class EndgameCloseClawSingleCommand extends CommandBase{

    //-------- DECLARATIONS --------\\

    private final EndgamePistonSubsystem piston;

    //-------- CONSTRUCTOR --------\\
    /**
     * EndgameCloseClawSingleCommand
     * 
     * @param pistonSubsystem the piston you want to close
     */
    public EndgameCloseClawSingleCommand(EndgamePistonSubsystem pistonSubsystem) {
        piston = pistonSubsystem;
        addRequirements(pistonSubsystem);
    }

    //-------- CLASS METHODS  --------\\
    
    public void initialize() { // runs once when called
        piston.closed();
    }
    
    //Leave false for default command
    @Override
    public boolean isFinished() { // when true, ends command
       return false;
    }
} // End of class EndgameCloseClawCommand
