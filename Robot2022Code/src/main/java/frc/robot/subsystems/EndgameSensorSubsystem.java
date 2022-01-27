package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3>EndgameSensorSubsystem</h3>
 * 
 * Manages sensor on endgame claw.
 */
public class EndgameSensorSubsystem extends SubsystemBase {

    // -------- DECLARATIONS --------\\

    private final DigitalInput sensor;

    // -------- CONSTRUCTOR --------\\

     /**
     * This constructor initializes the endgame sensors to the proper hardware
     * 
     * @param dioPort ID for the sensor
     */
    public EndgameSensorSubsystem (int dioPort) {
        sensor = new DigitalInput(dioPort);
    }

    // -------- METHODS --------\\

    /**
     * <h3>getSensorValue</h3>
     * This method returns the sensor's value
     * 
     * @return the value of the sensor
     */
    public boolean isTouching() {
        return !sensor.get();
    }
}
