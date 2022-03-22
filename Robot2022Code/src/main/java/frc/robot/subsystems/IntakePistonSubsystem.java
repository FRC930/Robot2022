//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//----- CLASS -----\\
/**
 * <h3>IntakePistonSubsystem</h3>
 * 
 * Handles the piston that raises and lowers the intake.
 */
public class IntakePistonSubsystem extends SubsystemBase {

    //----- SOLENOIDS -----\\

    private final Solenoid m_solenoid;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>IntakePistonSubsystem</h3>
     * 
     * Handles the pistons that raise and lower the intake.
     * 
     * @param solenoidID ID for the first intake solenoid that retracts
     */
    public IntakePistonSubsystem(int solenoidID) {

        //REVPH is the device on the robot that controls the compressors
        m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, solenoidID);
        
        //solenoid is being set to false to start
        m_solenoid.set(false);
    }

    /**
     * <h3>setIntakePistonState</h3>
     * 
     * Sets the shifter to the passed state
     * 
     * @param state the state to write to the piston
     */
    public void setIntakePistonState(boolean state) {
        m_solenoid.set(state);
    }

    /**
     * <h3>getIntakeSolenoidState</h3>
     * 
     * Reads the state of the intake piston
     * 
     * @return the piston state
     */
    public boolean getIntakeSolenoidOneState() {
        return m_solenoid.get();
    }

}