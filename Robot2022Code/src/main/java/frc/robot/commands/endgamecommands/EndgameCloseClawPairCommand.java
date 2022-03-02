/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameCloseClawPairCommand</h3>
 * 
 * Closes one mirror set of the endgame claws
 */
public class EndgameCloseClawPairCommand extends CommandBase{

    //-------- VARIABLES --------\\

    private final EndgamePistonSubsystem pistonLeft;
    private final EndgamePistonSubsystem pistonRight;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameCloseClawPairCommand</h3>
     * 
     * Closes one mirror set of the endgame claws
     * 
     * @param pistonLeft the piston on the left to close
     * @param pistonRight the mirror piston on the right to close
     */
    public EndgameCloseClawPairCommand(EndgamePistonSubsystem pistonLeft, EndgamePistonSubsystem pistonRight) {
        this.pistonLeft = pistonLeft;
        this.pistonRight = pistonRight;
        addRequirements(this.pistonLeft, this.pistonRight);
    }

    //-------- METHODS  --------\\
    
    public void initialize() { // runs once when called
        pistonLeft.closed();
        pistonRight.closed();
    }
    
    //Leave false for default command
    @Override
    public boolean isFinished() { // when true, ends command
       return false;
    }
} // End of class EndgameCloseClawPairCommand