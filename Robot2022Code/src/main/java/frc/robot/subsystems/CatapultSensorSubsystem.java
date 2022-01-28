package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3>CatapultSensorSubsystem</h3>
 * Controls the sensor for the catapult.
 */
public class CatapultSensorSubsystem extends SubsystemBase{
    private DigitalInput catapultSensor;

    /**
     * <h3>CatapultSensorSubsystem</h3>
     * 
     * Initializes a new {@link frc.robot.subsystems.CatapultSensorSubsystem
     * CatapultSensorSubsystem} with the passed DIO port.
     * 
     * @param portDio the port of the sensor
     */
    public CatapultSensorSubsystem(int portDio){
        catapultSensor = new DigitalInput(portDio);
    }

    /**
    * <h3>getDigitalInput</h3>
    * Returns the catapultSensor.
    */
    public DigitalInput getDigitalInput(){
        return catapultSensor;
    }

    /**
    * <h3>isActivated</h3>
    * Returns true if the sensor detects a ball.
    * @return true or false
    */
    public boolean isActivated() {
        return catapultSensor.get();
    }
}
