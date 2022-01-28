//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShuffleboardUtility;

import static frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

//----- CLASS -----\\
/**
 * <h3>Intake Piston Subsystem</h3>
 * 
 * Handles the pistons that raise and lower the intake.
 */
public class IntakePistonSubsystem extends SubsystemBase {

    // ----- SOLENOID(S) -----\\

    private Solenoid solenoidOne;
    private Solenoid solenoidTwo;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>Intake Piston Subsystem</h3>
     * 
     * Handles the pistons that raise and lower the intake.
     * 
     * @param solenoidOneID ID for the first intake solenoid.
     * @param solenoidTwoID ID for the second intake solenoid.
     */
    public IntakePistonSubsystem(int solenoidOneID, int solenoidTwoID) {
        solenoidOne = new Solenoid(PneumaticsModuleType.REVPH, solenoidOneID);
        solenoidTwo = new Solenoid(PneumaticsModuleType.REVPH, solenoidTwoID);
    }

    /**
     * <h3>setIntakePistonState</h3>
     * 
     * Sets the shifter to the passed state
     * 
     * @param state the state to write to the piston
     */
    public void setIntakePistonState(boolean state) {
        solenoidOne.set(state);
        solenoidTwo.set(state);

        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab, ShuffleboardKeys.INTAKE_DOWN,
                new ShuffleboardUtility.ShuffleBoardData<Boolean>(state));
    }

    /**
     * <h3>getIntakeSolenoidOneState</h3>
     * 
     * Reads the state of the second intake piston
     * 
     * @return the piston state
     */
    public boolean getIntakeSolenoidOneState() {
        return solenoidOne.get();
    }

    /**
     * <h3>getIntakeSolenoidTwoState</h3>
     * 
     * Reads the state of the second intake piston
     * 
     * @return the piston state
     */
    public boolean getIntakeSolenoidTwoState() {
        return solenoidTwo.get();
    }

}