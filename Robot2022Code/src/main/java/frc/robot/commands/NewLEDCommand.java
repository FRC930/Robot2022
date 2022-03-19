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
    private final int FLASH_TIMER = 5;
    // Used for moving segments (endgame pattern)
    private final int ENDGAME_TIMER = 1;
    // Used for retraction patterns
    private final int RETRACT_TIMER = 1;

    // Length of a singular segment
    private final int SEGMENT_LENGTH = 20;

    // Starting index for each strip
    // This is needed due to the uneven distribution of LEDs on the week 1 robot
    // (lost one at the beginning)
    // Order is as follows: Front Left, Back Left, Front Right, Back Right, Total #
    // of indexes
    private final int STRAND_STARTS[] = { 0, 74, 149, 224, 299 };

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

    // Conditions
    private boolean m_hasFlashedTop;
    private boolean m_hasFlashedBottom;
    private boolean m_hadTwoBalls;
    private boolean m_hadOneBall;

    //-----  LED BUFFERS  -----\\

    // Virtual placeholder for the LED strip
    // Compile data is sent to the LED strip through the subsystem
    private final AddressableLEDBuffer m_fullBuffer;
    private AddressableLEDBuffer m_singleStrandBuffer;

    // Precompiled buffers for solid patterns
    private final AddressableLEDBuffer m_offBuffer;
    private final AddressableLEDBuffer m_yellowBuffer;
    private final AddressableLEDBuffer m_greenBuffer;
    private AddressableLEDBuffer m_fullAllianceColor;
    private AddressableLEDBuffer m_halfAllianceColor;

    //----- MISCELLANEOUS -----\\

    // The LED Subsystem (strip) itself
    private final LEDSubsystem m_LEDSubsystem;

    // LED State to use
    private LEDStates m_LEDState;    

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>LEDCommand</h3>
     * 
     * Manages and deploys LED patterns.
     */
    public NewLEDCommand(LEDSubsystem leds, ControllerManager driverController) {

        m_LEDState = LEDStates.Disabled;
        
        m_LEDSubsystem = leds;
        m_fullBuffer = m_LEDSubsystem.getBuffer();
        m_singleStrandBuffer = new AddressableLEDBuffer((int) Math.ceil(m_fullBuffer.getLength() / 4.0));

        m_offBuffer = createFullColoredBuffer(black);
        m_yellowBuffer = createFullColoredBuffer(yellow);
        m_greenBuffer = createFullColoredBuffer(green);
        m_fullAllianceColor = createFullColoredBuffer((allianceColor == Alliance.Blue) ? blue : red);
        m_halfAllianceColor = createBottomColoredBuffer((allianceColor == Alliance.Blue) ? blue : red);

        m_hadOneBall = false;
        m_hadTwoBalls = false;
        m_hasFlashedBottom = false;
        m_hasFlashedTop = false;

    }

    //----- COMMAND METHODS -----\\

    /**
     * <h3>initialize</h3>
     */
    @Override
    public void initialize() {
        switch(m_LEDState) {
            case Disabled:
                
                break;
            case Autonomous:

                break;
            default:

                break;
        }

    }

    /**
     * <h3>execute</h3>
     */
    @Override
    public void execute() {
        switch(m_LEDState) {
            case Teleoperated:

                break;
            case Endgame:

                break;
            default:

                break;
        }
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

    private AddressableLEDBuffer createBottomColoredBuffer(Color8Bit color) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_singleStrandBuffer.getLength());

        for(int i = 0; i < buffer.getLength()/2; i++) {
            buffer.setLED(i, color);
        }

        return buffer;
    }

    //----- ACTION METHODS -----\\

    /**
     * <h3>movingSegmentPattern</h3>
     * Shifts a segment of LEDs along the strip.
     * pattern for Endgame
     */
    private void movingSegmentPattern() {
        m_counter++;
        if (m_counter >= m_singleStrandBuffer.getLength() * ENDGAME_TIMER) {
            m_counter = 0;
        }
        // Keeping track of animation speed.
        if (m_counter % ENDGAME_TIMER == 0) {

            if (m_beamStartPosition > m_singleStrandBuffer.getLength()) {
                m_beamStartPosition = 0;
            }

            if (m_beamEndPosition > m_singleStrandBuffer.getLength()) {
                m_beamEndPosition = 0;
            }

            if (m_beamStartPosition >= 0) {
                m_singleStrandBuffer.setLED(m_beamStartPosition, black);
            }

            m_singleStrandBuffer.setLED(m_beamEndPosition, (allianceColor == Alliance.Blue) ? blue : red);

            m_beamStartPosition += 1;
            m_beamEndPosition += 1;

           applyBuffer();
        }
    }

    /**
     * <h3>flashLEDHighPattern</h3>
     * 
     * Flashes alliance color on the top half three times then remains on.
     * 
     * @param num - Number of times to flash
     */
    private void flashTop(int num) {
        
        if(m_counter == FLASH_TIMER * 2 * num) { // If flashed num times
            m_hasFlashedTop = true;
        } else if(m_counter % (FLASH_TIMER * 2) < FLASH_TIMER) { // If needs to flash again
            m_singleStrandBuffer = m_halfAllianceColor;
        } else {
            m_singleStrandBuffer = m_fullAllianceColor;
        }

        applyBuffer();

        m_counter++;
    }

    /**
     * <h3>flashBottom</h3>
     * 
     * Flashes the bottom half of LEDs num times.
     */
    private void flashBottom(int num) {

        if(m_counter == FLASH_TIMER * 2 * num) { // If flashed num times
            m_hasFlashedBottom = true;
        } else if(m_counter % (FLASH_TIMER * 2) < FLASH_TIMER) { // If needs to flash again
            m_singleStrandBuffer = m_halfAllianceColor;
        } else {
            m_singleStrandBuffer = m_fullAllianceColor;
        }

        applyBuffer();

        m_counter++;
    }

    //----- STATE METHODS -----\\

    public void setLEDState(LEDStates newState) {
        m_LEDState = newState;
    }
    
    private void disabledState() {
        m_singleStrandBuffer = m_yellowBuffer;
    }

    private void autonomousState() {

    }

    private void teleoperatedState() {

    }

    private void endgameState() {

    }

    //----- ENUMS -----\\

    /**
     * <h3>LEDStates</h3>
     */
    public static enum LEDStates {
        Disabled, Autonomous, Teleoperated, Endgame;
    }
}
