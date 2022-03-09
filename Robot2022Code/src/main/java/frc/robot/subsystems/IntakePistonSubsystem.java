//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//----- CLASS -----\\
/**
 * <h3>IntakePistonSubsystem</h3>
 * 
 * Handles the pistons that raise and lower the intake.
 */
public class IntakePistonSubsystem extends SubsystemBase {

    //----- SOLENOIDS -----\\

    private final Solenoid m_solenoidOne;
    private final Solenoid m_solenoidTwo;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>IntakePistonSubsystem</h3>
     * 
     * Handles the pistons that raise and lower the intake.
     * 
     * @param solenoidOneID ID for the first intake solenoid that retracts
     * @param solenoidTwoID ID for the second intake solenoid that always stays open
     */
    public IntakePistonSubsystem(int solenoidOneID, int solenoidTwoID) {

        //REVPH is the device on the robot that controls the compressors
        m_solenoidOne = new Solenoid(PneumaticsModuleType.REVPH, solenoidOneID);
        m_solenoidTwo = new Solenoid(PneumaticsModuleType.REVPH, solenoidTwoID);
        
        //solenoidTwo is being set to true so that way there is room for another ball
        m_solenoidTwo.set(true);
    }

    /**
     * <h3>setIntakePistonState</h3>
     * 
     * Sets the shifter to the passed state
     * 
     * @param state the state to write to the piston
     */
    public void setIntakePistonState(boolean state) {

        //Sets the first solenoid to the boolean passed in and sets the second solenoid to the opposite state
        m_solenoidOne.set(state);
        m_solenoidTwo.set(!state);
    }

    /**
     * <h3>getIntakeSolenoidOneState</h3>
     * 
     * Reads the state of the second intake piston
     * 
     * @return the piston state
     */
    public boolean getIntakeSolenoidOneState() {
        return m_solenoidOne.get();
    }

}