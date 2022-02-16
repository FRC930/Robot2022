package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>BallSensorUtility</h3>
 * Controls the sensors used for the balls.
 */
public class BallSensorUtility {
    
    private static BallSensorUtility instance = null;

    public static BallSensorUtility getInstance(){
        if(instance == null){
            instance = new BallSensorUtility();
        }
        return instance;
    }

    private final int CATAPULTID = 0;
    private final int INDEXERID = 5;

    private final DigitalInput catapultSensor;
    private final DigitalInput indexerSensor;

    private BallSensorUtility(){
        catapultSensor = new DigitalInput(CATAPULTID);
        indexerSensor = new DigitalInput(INDEXERID);
    }


    /**
    * <h3>catapultIsTripped</h3>
    * Returns true if the catapult sensor detects a ball.
    * @return if the sensor was tripped
    */
    public boolean catapultIsTripped() {
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, ShuffleboardKeys.CATAPULT_SENSOR, new ShuffleBoardData<Boolean>(!catapultSensor.get()));
        return !catapultSensor.get();
    }

    /**
    * <h3>indexerIsTripped</h3>
    * Returns true if the indexer sensor detects a ball.
    * @return if the sensor was tripped
    */
    public boolean indexerIsTripped() {
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, ShuffleboardKeys.INTAKE_SENSOR, new ShuffleBoardData<Boolean>(!indexerSensor.get()));
        return !indexerSensor.get();
    }
}
