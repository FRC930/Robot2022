//----- IMPORTS  -----\\

package frc.robot.commands;

import frc.robot.ControllerManager;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BallSensorUtility;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

//----- CLASS  -----\\
/**
 * <h3>LEDCommand</h3>
 * 
 * Manages and deploys LED patterns.
 */
public class NewLEDCommand extends CommandBase {

    //-----  CONSTANTS  -----\\

    // Timer Counters
    // Used for flash patterns
    private static final int FLASH_TIMER = 5;
    // Used for moving segments (endgame pattern)
    private static final int ENDGAME_TIMER = 1;
    // Used for retraction patterns
    private static final int RETRACT_TIMER = 1;

    // Length of a singular segment
    private static final int SEGMENT_LENGTH = 20;

    // Starting index for each strip
    // This is needed due to the uneven distribution of LEDs on the week 1 robot
    // (lost one at the beginning)
    // Order is as follows: Front Left, Back Left, Front Right, Back Right, Total #
    // of indexes
    private static final int STRAND_STARTS[] = { 0, 74, 149, 224, 299 };

    //-----  COLORS  -----\\

    private static final Color8Bit black = new Color8Bit(0, 0, 0);
    private static final Color8Bit white = new Color8Bit(80, 80, 80);
    private static final Color8Bit red = new Color8Bit(125, 0, 0);
    private static final Color8Bit yellow = new Color8Bit(55, 40, 0);
    private static final Color8Bit green = new Color8Bit(0, 125, 0);
    private static final Color8Bit blue = new Color8Bit(0, 0, 125);
    // Current Alliance color
    private Alliance allianceColor;

    //-----  VARIABLES  -----\\

    // Flag to determine time delays
    // One execute() cycle is 0.020 seconds
    private int m_counter = 0;

    private int m_beamStartPosition;
    private int m_beamEndPosition;

    private boolean m_aimIsPressed;

    //-----  LED BUFFERS  -----\\

    // Virtual placeholder for the LED strip
    // Compile data is sent to the LED strip through the subsystem
    private final AddressableLEDBuffer m_fullBuffer;
    private final AddressableLEDBuffer m_singleStrandBuffer;
    // Precompiled buffers for solid patterns
    private final AddressableLEDBuffer m_OffBuffer;
    private final AddressableLEDBuffer m_YellowBuffer;
    private final AddressableLEDBuffer m_GreenBuffer;
    private AddressableLEDBuffer m_allianceColor;

    //----- MISCELLANEOUS -----\\

    // The LED Subsystem (strip) itself
    private final LEDSubsystem m_LEDSubsystem;
    // manages the last ball status
    private ballStatus m_lastBallStatus;
    // manages the current ball statue
    private ballStatus m_currentBallStatus;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>LEDCommand</h3>
     * 
     * Manages and deploys LED patterns.
     */
    public NewLEDCommand(LEDSubsystem leds, ControllerManager driverController) {
        
        m_LEDSubsystem = leds;
        m_fullBuffer = m_LEDSubsystem.getBuffer();
        m_singleStrandBuffer = new AddressableLEDBuffer((int) Math.ceil(m_fullBuffer.getLength() / 4.0));

        m_OffBuffer = createFullColoredBuffer(black);
        m_YellowBuffer = createFullColoredBuffer(yellow);
        m_GreenBuffer = createFullColoredBuffer(green);
        m_allianceColor = createFullColoredBuffer((allianceColor == Alliance.Blue) ? blue : red);
       
    }

    //----- COMMAND METHODS -----\\

    /**
     * <h3>initialize</h3>
     */
    @Override
    public void initialize() {

    }

    /**
     * <h3>execute</h3>
     */
    @Override
    public void execute() {

    }

    /**
     * <h3>isFinished</h3>
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    //----- HARDWARE METHODS -----\\

    private void applyBuffer() {
        for (int j = 0; j < 4; j++) {
            for (int i = 0; i < m_singleStrandBuffer.getLength(); i++) {
                // Stop if beyond side
                if ((i + STRAND_STARTS[j]) == STRAND_STARTS[j + 1]) {
                    break;
                }
                // Use for back sides
                if (j % 2 == 1) {
                    m_fullBuffer.setLED(STRAND_STARTS[j + 1] - (i + 1), m_singleStrandBuffer.getLED8Bit(i));
                } else {
                    m_fullBuffer.setLED(STRAND_STARTS[j] + i, m_singleStrandBuffer.getLED8Bit(i));
                }
            }
        }
        m_LEDSubsystem.setBuffer(m_fullBuffer);
    }

    //----- CREATE METHODS -----\\

    private AddressableLEDBuffer createFullColoredBuffer(Color8Bit color) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_singleStrandBuffer.getLength());

        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }

        return buffer;
    }

    
    

    //----- ACTION METHODS -----\\

    //----- STATE METHODS -----\\
    


    //----- ENUMS -----\\

    /**
     * <h3>LEDStates</h3>
     */
    public static enum LEDStates {
        Disabled, Autonomous, Teleoperated, Endgame;
    }

    /**
     * <h3>ballStatus</h3>
     * 
     * Flag to determine state change for teleoperated status.
     */
    private enum ballStatus {
        OneBall, TwoBalls, NoBall;
    }

}
