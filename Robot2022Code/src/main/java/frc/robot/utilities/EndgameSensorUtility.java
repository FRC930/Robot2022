package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalInput;

public class EndgameSensorUtility {

    private static EndgameSensorUtility instance = null;

    public static EndgameSensorUtility getInstance(){
        if(instance == null){
            instance = new EndgameSensorUtility();
        }
        return instance;
    }

    private final int LEFT2ID = 1;
    private final int RIGHT2ID = 2;
    private final int LEFT4ID = 3;
    private final int RIGHT4ID = 4;

    private final DigitalInput sensorL2;
    private final DigitalInput sensorR2;
    private final DigitalInput sensorL4;
    private final DigitalInput sensorR4;

    private EndgameSensorUtility(){
        sensorL2 = new DigitalInput(LEFT2ID);
        sensorR2 = new DigitalInput(RIGHT2ID);
        sensorL4 = new DigitalInput(LEFT4ID);
        sensorR4 = new DigitalInput(RIGHT4ID);
    }

    // -------- METHODS --------\\
    /*NOTE: DIO is opposite of sensor contact
    *DIO is 0 when sensor is activated(touching metal)
    */

    /**
     * <h3>left2IsTouching</h3>
     * This method returns the left 2 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean left2IsTouching() {
        return !sensorL2.get();
    }
    
    /**
     * <h3>right2IsTouching</h3>
     * This method returns the right 2 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean right2IsTouching() {
        return !sensorR2.get();
    }
    
    /**
     * <h3>left4IsTouching</h3>
     * This method returns the left 4 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean left4IsTouching() {
        return !sensorL4.get();
    }

    /**
     * <h3>right4IsTouching</h3>
     * This method returns the right 4 sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean right4IsTouching() {
        return !sensorR4.get();
    }
}
