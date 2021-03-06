/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

//----- CLASS -----\\
/**
 * <h3>EndgamePistonSubsystem</h3>
 *
 * manages individual piston on endgame grabber
 */
public class EndgamePistonSubsystem extends SubsystemBase{

    //----- SOLENOIDS -----\\

    private final Solenoid m_grabberPiston;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>EndgamePistonSubsystem</h3>
     *
     * Constructor for endgame piston
     * 
     * @param solenoid_ID id of solenoid port
     */
    public EndgamePistonSubsystem(int solenoid_ID) {
        
        //CTRE pneumatic hub has 8 slots. Cap is placed on simulation to prevent errors.
        m_grabberPiston = new Solenoid(
            Robot.isReal()||solenoid_ID>7?PneumaticsModuleType.REVPH:PneumaticsModuleType.CTREPCM, 
            solenoid_ID);
    }

    //----- METHODS -----\\

    /**
     * <h3>toggle</h3>
     *
     * changes grabber position. If true sets false, 
     * if false set true
     * 
     */
    public void toggle() {
        m_grabberPiston.set(!m_grabberPiston.get());
    }

    /**
     * <h3>isOpen</h3>
     *
     * method returns if piston is open
     * 
     * @return grabber piston state
     */
    public boolean isOpen() {
        return m_grabberPiston.get();
    }

    /**
     * <h3>open</h3>
     *
     * sets piston to open
     */
    public void open() {
        m_grabberPiston.set(true);
    }
    /**
     * <h3>closed</h3>
     *
     * sets piston to closed
     */
    public void closed() {
        m_grabberPiston.set(false);
    }

} // End of class EndgamePistonSubsystem