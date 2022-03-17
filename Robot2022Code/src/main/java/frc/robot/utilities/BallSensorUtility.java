package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/**
 * <h3>BallSensorUtility</h3>
 * Controls the sensors used for the balls.
 */
public class BallSensorUtility {
    //-----CONSTANTS----\\
    private final int TRIGGER_DISTANCE = 100;
    private final int INTAKE_ID = 16;
    private final int LOADED_ID = 15;
    
    private static BallSensorUtility instance = null;

    public static BallSensorUtility getInstance(){
        if(instance == null){
            instance = new BallSensorUtility();
        }
        return instance;
    }

    private final TimeOfFlight intakeSensor;
    private final TimeOfFlight loadedSensor;

    private BallSensorUtility(){
        intakeSensor = new TimeOfFlight(INTAKE_ID);
        loadedSensor = new TimeOfFlight(LOADED_ID);
        intakeSensor.setRangingMode(RangingMode.Short, 25);
        loadedSensor.setRangingMode(RangingMode.Short, 25);
    }


    /**
    * <h3>intakeIsTripped</h3>
    * Returns true if the intake sensor detects a ball.
    * @return if the sensor was tripped
    */
    public boolean intakeIsTripped() {
        //System.out.println("Intake Sensor: " + intakeSensor.getRange());
        return intakeSensor.getRange() < TRIGGER_DISTANCE;
    }

    /**
    * <h3>loadedIsTripped</h3>
    * Returns true if the loaded sensor detects a ball.
    * @return if the sensor was tripped
    */
    public boolean loadedIsTripped() {
        //System.out.println("Loaded Sensor: " + loadedSensor.getRange());
        return loadedSensor.getRange() < TRIGGER_DISTANCE;
    }
}
