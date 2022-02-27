/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalInput;

//----- CLASS -----\\
/**
 * <h3>EndgameSensorUtility</h3>
 * 
 * Creates and returns sensor values the endgame sensors
 */
public class EndgameSensorUtility {

    //-------- CONSTANTS --------\\

    private static final int LEFT_2_ID = 1;
    private static final int RIGHT_2_ID = 2;
    private static final int LEFT_4_ID = 3;
    private static final int RIGHT_4_ID = 4;
    
    //-------- VARIABLES --------\\
    private static EndgameSensorUtility instance = null;
    private final DigitalInput sensorL2;
    private final DigitalInput sensorR2;
    private final DigitalInput sensorL4;
    private final DigitalInput sensorR4;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>EndgameSensorUtility</h3>
     * 
     * Creates and returns sensor values the endgame sensors
     */
    private EndgameSensorUtility() {
        sensorL2 = new DigitalInput(LEFT_2_ID);
        sensorR2 = new DigitalInput(RIGHT_2_ID);
        sensorL4 = new DigitalInput(LEFT_4_ID);
        sensorR4 = new DigitalInput(RIGHT_4_ID);
    }

    //-------- METHODS --------\\

    /**
     * <h3>getInstance</h3>
     * 
     * Gets the instance of the endgame sensor
     * 
     * @return the instance of the endgame sensor
     */
    public static EndgameSensorUtility getInstance() {
        if (instance == null) {
            instance = new EndgameSensorUtility();
        }
        return instance;
    }

    /**
     * NOTE: DIO is opposite of sensor contact
     * DIO is 0 when sensor is activated(touching metal)
     */

    /**
     * <h3>left2IsTouching</h3>
     * 
     * This method returns the left 2 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean left2IsTouching() {
        return !sensorL2.get();
    }

    /**
     * <h3>right2IsTouching</h3>
     * 
     * This method returns the right 2 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean right2IsTouching() {
        return !sensorR2.get();
    }

    /**
     * <h3>left4IsTouching</h3>
     * 
     * This method returns the left 4 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean left4IsTouching() {
        return !sensorL4.get();
    }

    /**
     * <h3>right4IsTouching</h3>
     * 
     * This method returns the right 4 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean right4IsTouching() {
        return !sensorR4.get();
    }

} // End of class EndgameSensorUtility