package frc.robot.utilities;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

    private final Debouncer sensorDebounce = new Debouncer(0.1, DebounceType.kRising);

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
     * <h3>catapultIsReset</h3>
     * 
     * Get whether the catapult has returned to rest position
     * 
     * @return the state of the catapult return sensor
     */
    public boolean catapultIsReset() {
        return sensorDebounce.calculate(!catapultReturnSensor.get());
    }
}
