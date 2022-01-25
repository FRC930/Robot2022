package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Robot;

/**
 * <h3> EndgamePistonSubsystem </h3>
 * manages individual piston on endgame grabber
 */
public class EndgamePistonSubsystem {
    Solenoid grabberPiston;

    /**
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
    /**
     * changes grabber position. If true sets false, 
     * if false set true
     * 
     */
    public void toggle() {
        if (grabberPiston.get())
            grabberPiston.set(false);
        else 
            grabberPiston.set(true);
    }
    /**
     * method returns if piston is open
     * 
     * @return grabber piston state
     */
    public boolean isOpen() {
        return grabberPiston.get();
    }
    /**
     * sets piston to open
     */
    public void open() {
        grabberPiston.set(true);
    }
    /**
     * sets piston to closed
     */
    public void closed() {
        grabberPiston.set(false);
    }

}