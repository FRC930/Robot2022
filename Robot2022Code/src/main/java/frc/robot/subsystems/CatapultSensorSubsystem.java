package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3>CatapultSensorSubsystem</h3>
 * Controls the sensor for the catapult.
 */
public class CatapultSensorSubsystem extends SubsystemBase{
    public DigitalInput sensor;
    

    /**
     * <h3>CatapultSensorSubsystem</h3>
     * 
     * Initializes a new {@link frc.robot.subsystems.CatapultSensorSubsystem
     * CatapultSensorSubsystem} with the passed DIO port.
     * 
     * @param portDio the port of the sensor
     */
    public CatapultSensorSubsystem(int sensorPort){
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
