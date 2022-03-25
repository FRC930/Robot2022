/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.EndgameSensorUtility;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameEndWhenTouching</h3>
 * 
 * Command that ends when the sensor triggers
 */
public class EndgameEndWhenTouching extends CommandBase {
    
    //-------- VARIABLES --------\\
    private final int sensor;
    private final EndgameSensorUtility sensorUtility = EndgameSensorUtility.getInstance();

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameEndWhenTouching</h3>
     * 
     * Ends command when sensor triggers
     * 
     * @param sensorSet sensor pair used to detect
     */
    public EndgameEndWhenTouching(int sensorSet){
        sensor = sensorSet;
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
} // End of class EndgameCloseWhenTouching