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
import frc.robot.utilities.EndgameSensorUtility;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameCloseWhenTouching</h3>
 * 
 * Closes a claw when the sensor is active
 */
public class EndgameCloseWhenTouching extends CommandBase {
    
    //-------- VARIABLES --------\\
    
    private final EndgamePistonSubsystem endgamePiston;
    private final int sensor;
    private final EndgameSensorUtility sensorUtility = EndgameSensorUtility.getInstance();

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameCloseWhenTouching</h3>
     * 
     * Closes a claw when the sensor is active
     * 
     * @param _endgamePiston piston of claw to be closed
     * @param _endgameSensor sensor used to detect
     */
    public EndgameCloseWhenTouching(EndgamePistonSubsystem _endgamePiston, int sensorSet){
        sensor = sensorSet;
        endgamePiston = _endgamePiston;
        addRequirements(endgamePiston);
    }

    //-------- METHODS --------\\

    @Override
    public boolean isFinished() { // returns true when the sensor is active
        if(sensor == 2){
            return sensorUtility.left2IsTouching() && 
            sensorUtility.right2IsTouching();
        }
        else if(sensor == 4){
            return sensorUtility.left4IsTouching() && 
            sensorUtility.right4IsTouching();
        }
        else{
            return true;
        }
    }

    @Override // Interrupted when button is released
    public void end(boolean interuppted) { // closes the claw
        endgamePiston.closed();
    }

} // End of class EndgameCloseWhenTouching