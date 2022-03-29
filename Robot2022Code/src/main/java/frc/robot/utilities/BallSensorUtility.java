package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

/**
 * <h3>BallSensorUtility</h3>
 * Controls the sensors used for the balls.
 */
public class BallSensorUtility {

    // -----CONSTANTS----\\

    private final int TRIGGER_DISTANCE = 200;
    private final int INTAKE_ID = 16;
    private final int LOADED_ID = 15;

    private static BallSensorUtility instance = null;

    public static BallSensorUtility getInstance() {
        if (instance == null) {
            instance = new BallSensorUtility();
        }
        return instance;
    }

    private TimeOfFlight intakeSensor;
    private TimeOfFlight loadedSensor;
    private DigitalInput intakeSensorSim;
    private DigitalInput loadedSensorSim;

    private BallSensorUtility() {
        if (Robot.isReal()) {
            intakeSensor = new TimeOfFlight(INTAKE_ID);
            loadedSensor = new TimeOfFlight(LOADED_ID);
            intakeSensor.setRangingMode(RangingMode.Short, 25);
            loadedSensor.setRangingMode(RangingMode.Short, 25);
        } else {
            intakeSensorSim = new DigitalInput(5);
            loadedSensorSim = new DigitalInput(6);
        }
    }

    /**
     * <h3>intakeIsTripped</h3>
     * Returns true if the intake sensor detects a ball.
     * 
     * @return if the sensor was tripped
     */
    public boolean intakeIsTripped() {
        if (Robot.isReal()) {
            return intakeSensor.getRange() < TRIGGER_DISTANCE;
        } else {
            return intakeSensorSim.get();
        }
    }

    /**
     * <h3>loadedIsTripped</h3>
     * Returns true if the loaded sensor detects a ball.
     * 
     * @return if the sensor was tripped
     */
    public boolean loadedIsTripped() {
        if (Robot.isReal()) {
            return loadedSensor.getRange() < TRIGGER_DISTANCE;
        } else {
            return loadedSensorSim.get();
        }
    }
}
