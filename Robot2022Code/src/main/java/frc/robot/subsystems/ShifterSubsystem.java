//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShifterUtility;

//----- CLASS -----\\
/**
 * <h3>ShifterSubsystem</h3>
 * 
 * Controls the solenoid for the shifting drivetrain
 */
public class ShifterSubsystem extends SubsystemBase {

    //----- SOLENOIDS -----\\

    private final Solenoid m_shifterSolenoid;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>ShifterSubsystem</h3>
     * 
     * Initializes a new {@link frc.robot.subsystems.ShifterSubsystem
     * ShifterSubsystem} with the passed solenoid ID.
     * 
     * @param solenoidID    - the ID of the solenoid
     */
    public ShifterSubsystem(int solenoidID) {
        m_shifterSolenoid = new Solenoid(PneumaticsModuleType.REVPH, solenoidID);
    }

    //----- METHODS -----\\

    /**
     * <h3>setShifterState</h3>
     * 
     * Sets the shifter to the passed state
     * 
     * @param state - the state to write to the piston
     */
    public void setShifterState(boolean state) {
        m_shifterSolenoid.set(state);
        ShifterUtility.setShifterState(state);
    }

    /**
     * <h3>getShifterState</h3>
     * 
     * Reads the state of the shifting piston
     * 
     * @return the piston state
     */
    public boolean getShifterState() {
        return m_shifterSolenoid.get();
    }
}
