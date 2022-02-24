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
 * <h3>EndgameCloseWhenAway</h3>
 * 
 * Closes a claw when the sensor is clear
 */
public class EndgameCloseWhenAway extends CommandBase {

    //-------- VARIABLES --------\\

    private final EndgamePistonSubsystem endgamePiston;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameCloseWhenAway</h3>
     * 
     * Closes a claw when the sensor is clear
     * 
     * @param _endgamePiston piston of claw to be closed
     * 
     */
    public EndgameCloseWhenAway(EndgamePistonSubsystem _endgamePiston) {
        endgamePiston = _endgamePiston;
        addRequirements(endgamePiston);
    }

    //-------- COMMANDBASE METHODS --------\\
    /*
    @Override
    public boolean isFinished() { // returns true when the sensor is clear
        return !endgameSensor.isTouching();
    }*/

    @Override
    public void end(boolean interuppted) { // closes the claw
        endgamePiston.closed();
    }

} // End of class EndgameCloseWhenAway