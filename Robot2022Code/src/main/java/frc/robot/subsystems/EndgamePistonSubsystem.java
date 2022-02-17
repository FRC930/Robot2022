//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h3>EndgamePistonSubsystem</h3>
 *
 * manages individual piston on endgame grabber
 */
public class EndgamePistonSubsystem extends SubsystemBase{
    Solenoid grabberPiston;

    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>EndgamePistonSubsystem</h3>
     *
     * Constructor for endgame piston
     * 
     * @param solenoid_ID id of solenoid port
     */
    public EndgamePistonSubsystem(int solenoid_ID) {
        //CTRE pneumatic hub has 8 slots. Cap is placed on simulation to prevent errors.
        grabberPiston = new Solenoid(
            Robot.isReal()||solenoid_ID>7?PneumaticsModuleType.REVPH:PneumaticsModuleType.CTREPCM, 
            solenoid_ID);
    }

    // -------- METHODS --------\\

    /**
     * <h3>toggle</h3>
     *
     * changes grabber position. If true sets false, 
     * if false set true
     * 
     */
    public void toggle() {
        grabberPiston.set(!grabberPiston.get());
    }

    /**
     * <h3>isOpen</h3>
     *
     * method returns if piston is open
     * 
     * @return grabber piston state
     */
    public boolean isOpen() {
        return grabberPiston.get();
    }

    /**
     * <h3>open</h3>
     *
     * sets piston to open
     */
    public void open() {
        grabberPiston.set(true);
    }
    /**
     * <h3>closed</h3>
     *
     * sets piston to closed
     */
    public void closed() {
        grabberPiston.set(false);
    }

}