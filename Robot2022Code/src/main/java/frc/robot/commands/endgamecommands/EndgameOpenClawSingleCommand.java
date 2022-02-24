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
 * <h3>EndgameOpenClawSingleCommand</h3>
 * 
 * Opens one of the endgame claws
 */
public class EndgameOpenClawSingleCommand extends CommandBase {

    //-------- VARIABLES --------\\

    private final EndgamePistonSubsystem piston;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameOpenClawSingleCommand</h3>
     * 
     * Opens one of the endgame claws
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

} // End of class EndgameOpenClawSingleCommand