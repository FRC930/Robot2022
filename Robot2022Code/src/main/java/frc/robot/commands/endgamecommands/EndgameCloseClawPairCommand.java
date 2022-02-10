package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
//-------- COMMAND CLASS --------\\
/**
 * <h3> EndgameCloseClawPairCommand</h3>
 * 
 * Closes one mirror set of the endgame claws
 */
public class EndgameCloseClawPairCommand extends CommandBase{

    //-------- DECLARATIONS --------\\

    private final EndgamePistonSubsystem pistonLeft;
    private final EndgamePistonSubsystem pistonRight;

    //-------- CONSTRUCTOR --------\\
    /**
     * EndgameCloseClawPairCommand
     * 
     * @param pistonLeft the piston on the left to close
     * @param pistonRight the mirror piston on the right to close
     */
    public EndgameCloseClawPairCommand(EndgamePistonSubsystem pistonLeft, EndgamePistonSubsystem pistonRight) {
        this.pistonLeft = pistonLeft;
        this.pistonRight = pistonRight;
        addRequirements(this.pistonLeft, this.pistonRight);
    }

    //-------- CLASS METHODS  --------\\
    
    public void initialize() { // runs once when called
        pistonLeft.closed();
        pistonRight.closed();
    }
    
    //Leave false for default command
    @Override
    public boolean isFinished() { // when true, ends command
       return false;
    }
} // End of class EndgameCloseClawCommand
