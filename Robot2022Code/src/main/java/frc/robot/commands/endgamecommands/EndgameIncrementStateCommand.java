/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameIncrementStateCommand</h3>
 * 
 * Increases state in EndgameManagerCommand
 */
public class EndgameIncrementStateCommand extends CommandBase {

    //-------- VARIABLES --------\\
    
    private final EndgameManagerCommand managerCommand;

    //-------- METHODS --------\\

    /**
     * <h3>EndgameIncrementStateCommand</h3>
     * 
     * Increases state in EndgameManagerCommand
     * 
     * @param mCommand EndgameManagerCommand object to use
     */
    public EndgameIncrementStateCommand(EndgameManagerCommand mCommand) {
        managerCommand = mCommand;
    }

    @Override // Called when the command is initially scheduled.
    public void initialize() { 
        managerCommand.nextState();
    }

    @Override
    public boolean isFinished() { // when true, ends command
        return true;
    }

} // End of class EndgameIncrementStateCommand