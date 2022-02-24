/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

//----- CLASS -----\\
/**
 * <h3>EndgameSensorUtility</h3>
 * 
 * Creates and returns sensor values the endgame sensors
 */
public class EndgameSensorUtility {

    private static EndgameSensorUtility instance = null;

    public static EndgameSensorUtility getInstance() {
        if (instance == null) {
            instance = new EndgameSensorUtility();
        }
        return instance;
    }

    //-------- VARIABLES --------\\

    private final int LEFT2ID = 1;
    private final int RIGHT2ID = 2;
    private final int LEFT4ID = 3;
    private final int RIGHT4ID = 4;

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
        sensorL2 = new DigitalInput(LEFT2ID);
        sensorR2 = new DigitalInput(RIGHT2ID);
        sensorL4 = new DigitalInput(LEFT4ID);
        sensorR4 = new DigitalInput(RIGHT4ID);
    }

    //-------- METHODS --------\\

    /**
    *NOTE: DIO is opposite of sensor contact
    *DIO is 0 when sensor is activated(touching metal)
    */

    /**
     * <h3>left2IsTouching</h3>
     * 
     * This method returns the left 2 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean left2IsTouching() {
        boolean sensorValue = !sensorL2.get();
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.ENDGAME_SENSOR1, new ShuffleBoardData<Boolean>(sensorValue));
        return sensorValue;
        // return !sensorL2.get();
    }

    /**
     * <h3>right2IsTouching</h3>
     * 
     * This method returns the right 2 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean right2IsTouching() {
        boolean sensorValue = !sensorR2.get();
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.ENDGAME_SENSOR2, new ShuffleBoardData<Boolean>(sensorValue));
        return sensorValue;
        // return !sensorR2.get();
    }

    /**
     * <h3>left4IsTouching</h3>
     * 
     * This method returns the left 4 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean left4IsTouching() {
        boolean sensorValue = !sensorL4.get();
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.ENDGAME_SENSOR3, new ShuffleBoardData<Boolean>(sensorValue));
        return sensorValue;
        // return !sensorL4.get();
    }

    /**
     * <h3>right4IsTouching</h3>
     * 
     * This method returns the right 4 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean right4IsTouching() {
        boolean sensorValue = !sensorR4.get();
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.ENDGAME_SENSOR4, new ShuffleBoardData<Boolean>(sensorValue));
        return sensorValue;
        // return !sensorR4.get();
    }

} // End of class EndgameSensorUtility