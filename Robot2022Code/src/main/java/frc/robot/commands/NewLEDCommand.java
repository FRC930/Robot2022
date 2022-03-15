//----- IMPORTS -----\\

package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BallSensorUtility;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

//----- CLASS -----\\
/**
 * <h3>LEDCommand</h3>
 * 
 * Manages and deploys LED patterns.
 */
public class NewLEDCommand extends CommandBase {
    
    //----- CONSTANTS -----\\

    // Used for flash patterns
    private static final int FLASH_TIMER = 5;
    // Used for moving segments (endgame pattern)
    private static final int ENDGAME_TIMER = 1;
    // Used for retraction patterns
    private static final int RETRACT_TIMER = 1;
    
    // Length of a singular segment 
    private static final int SEGMENT_LENGTH = 20;

    //----- COLORS -----\\

    //----- VARIABLES -----\\

    //----- LED BUFFERS -----\\

    public NewLEDCommand() {

    }

}
