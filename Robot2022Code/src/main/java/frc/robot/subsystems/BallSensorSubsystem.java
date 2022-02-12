package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * <h3>BallSensorSubsystem</h3>
 * Controls the sensors used for the balls.
 */
public class BallSensorSubsystem {
    public DigitalInput sensor;
    

    /**
     * <h3>BallSensorSubsystem</h3>
     * 
     * Initializes a new {@link frc.robot.subsystems.BallSensorSubsystem
     * BallSensorSubsystem} with the passed DIO port.
     * 
     * @param portDio the port of the sensor
     */
    public BallSensorSubsystem(int sensorPort){
        sensor = new DigitalInput(sensorPort);
    }

    /**
    * <h3>isTripped</h3>
    * Returns true if the sensor detects a ball.
    * @return if the sensor was tripped
    */
    public boolean isTripped() {
        return !sensor.get();
    }
}
