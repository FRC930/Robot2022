package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * <h3>CatapultReturnSensorUtility</h3>
 * 
 * CatapultReturnSensorUtility stores the catapult return sensor in a utility
 */
public class CatapultReturnSensorUtility {
    // The DIO port for the sensor
    private static final int CATAPULT_RETURN_SENSOR_ID = 6;

    // Our singleton instance
    private static CatapultReturnSensorUtility instance = null;

    // The digital input for our sensor
    private DigitalInput catapultReturnSensor;

    /**
     * <h3>CatapultReturnSensorUtility</h3>
     * 
     * Initialize the singleton
     */
    private CatapultReturnSensorUtility() {
        catapultReturnSensor = new DigitalInput(CATAPULT_RETURN_SENSOR_ID);
    }

    /**
     * <h3>getInstance</h3>
     * 
     * Get the instance of this singleton
     * 
     * @return the current instance of the singleton
     */
    public static CatapultReturnSensorUtility getInstance() {
        if (instance == null) {
            instance = new CatapultReturnSensorUtility();
        }
        return instance;
    }

    /**
     * <h3>isCatapultReturned</h3>
     * 
     * Get whether the catapult has returned to rest position
     * 
     * @return the state of the catapult return sensor
     */
    public boolean isCatapultReturned() {
        return !catapultReturnSensor.get();
    }
}
