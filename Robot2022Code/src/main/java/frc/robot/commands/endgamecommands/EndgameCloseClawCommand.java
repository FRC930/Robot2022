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
 * <h3>EndgameCloseClawSingleCommand</h3>
 * 
 * Closes one of the endgame claws
 */
public class EndgameCloseClawCommand extends CommandBase{

    //-------- VARIABLES --------\\

    private final EndgamePistonSubsystem piston;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameCloseClawSingleCommand</h3>
     * 
     * Closes one of the endgame claws
     * 
     * @param pistonSubsystem the piston you want to close
     */
    public EndgameCloseClawCommand(EndgamePistonSubsystem pistonSubsystem) {
        piston = pistonSubsystem;
        addRequirements(pistonSubsystem);
    }

    //-------- METHODS  --------\\
    
    public void initialize() { // runs once when called
        piston.closed();
    }
    
    //Leave false for default command
    @Override
    public boolean isFinished() { // when true, ends command
       return false;
    } 
} // End of class EndgameCloseClawSingleCommand