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

import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import frc.robot.utilities.ShuffleboardUtility;

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

    

    private enum ballStatus {
        oneBall, TwoBalls, noBall
    };
    // manages the last ball status
    private ballStatus m_lastBallStatus;
    // manages the current ball statue
    private ballStatus m_currentBallStatus;

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
    private final ControllerManager m_driverController;

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

        m_lastBallStatus = ballStatus.noBall;
        m_currentBallStatus = m_lastBallStatus;

        m_aimIsPressed = false;

        m_driverController = driverController;

        addRequirements(m_LEDSubsystem);
    }

    //----- COMMAND METHODS -----\\

    /**
     * <h3>initialize</h3>
     */
    @Override
    public void initialize() {
         
          // Wait to determine alliance for FMS signal
        // Alliance can change during simulation
        allianceColor = DriverStation.getAlliance();
        m_counter = 0;
        m_beamStartPosition = -SEGMENT_LENGTH;
        m_beamEndPosition = 0;

        switch(m_LEDState) {
            case Disabled:
                disabledState();
                break;
            case Autonomous:
                autonomousState();
                break; 
            default:
                off();
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
                off();
                teleoperatedState();
                break;
            case Endgame:
                endgameState();
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
     * <h3>retractBottom</h3>
     * 
     * retracts LEDs one by one from half to empty
     */
    private void retractBottom() {
        if (m_counter >= RETRACT_TIMER * m_singleStrandBuffer.getLength() / 2) {
            if (m_counter % RETRACT_TIMER == 0) {
                m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() / 2 - (m_counter / RETRACT_TIMER), black);
                
            }
            applyBuffer();
            m_counter++;
        } else {
            m_lastBallStatus = ballStatus.noBall;
            
        }
      
    }

    /**
     * <h3>retractTop</h3>
     * 
     * retracts LEDs one by one from full to half
     */
    private void retractTop() {
        if (m_counter >= RETRACT_TIMER * m_singleStrandBuffer.getLength() / 2) {
            if (m_counter % RETRACT_TIMER == 0) {
                m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() - (m_counter / RETRACT_TIMER + 1), black);
            }
            applyBuffer();
            m_counter++;
        } else {
            m_lastBallStatus = ballStatus.oneBall;
        }
        
    }

    
    /**
     * <h3>flashTop</h3>
     * Flashes alliance color on the top half three times then remains on
     */
    private void flashTop() {
        m_counter++;
        if (m_counter > FLASH_TIMER * 3) {
            m_lastBallStatus = ballStatus.TwoBalls;
            m_counter = 0;
        } else {
            if (m_counter % (FLASH_TIMER) == 0) {
                m_singleStrandBuffer = m_fullAllianceColor;
            } else if (m_counter % (FLASH_TIMER) == FLASH_TIMER / 2) {
                m_singleStrandBuffer = m_halfAllianceColor;
            }
        }
        applyBuffer();
    }

    /**
     * <h3>flashBottom</h3>
     * Flashes alliance color on the bottom half three times then remains on
     */
    private void flashBottom() {
        m_counter++;
        if (m_counter > FLASH_TIMER * 6) {
            m_lastBallStatus = ballStatus.oneBall;
            m_counter = 0; // alliance
        } else {
            if (m_counter % (FLASH_TIMER * 2) == 0) {
               m_singleStrandBuffer = m_halfAllianceColor;
            } else if (m_counter % (FLASH_TIMER * 2) == FLASH_TIMER) {
               m_singleStrandBuffer = m_offBuffer;
            }
        }
        applyBuffer();
    }
    /**
     * <h3>aimStatus</h3>
     * 
     */
    private boolean aimStatus() {
        if (ShuffleboardUtility.getInstance().getFromShuffleboard(ShuffleboardKeys.AIMED) == null) {
            return false;
        }
        if ((boolean) ShuffleboardUtility.getInstance().getFromShuffleboard(ShuffleboardKeys.AIMED).getData()) {
            m_singleStrandBuffer = m_greenBuffer;
            return true;
        } else {
            m_singleStrandBuffer = m_offBuffer;
        }
        return false;
    }

    /**
     * <h3>off</h3>
     * 
     * Turns the LED strip off.
     */
    private void off() {
        m_singleStrandBuffer = m_offBuffer;
        applyBuffer();
    }

    //----- STATE METHODS -----\\

    public void setLEDState(LEDStates newState) {
        m_LEDState = newState;
    }
    
    private void disabledState() {
        m_singleStrandBuffer = m_yellowBuffer;
        applyBuffer();
    }

    private void autonomousState() {
        m_singleStrandBuffer = m_fullAllianceColor;
        applyBuffer();
    }

    private void teleoperatedState() {
        
         if (m_driverController.getRightBumper().get() 
                && !m_driverController.getLeftBumper().get() && aimStatus()){
            
            m_lastBallStatus = ballStatus.noBall;
            m_aimIsPressed = true;
               
        } else if(m_aimIsPressed) {
            off();
            m_aimIsPressed = false;
        } else if (BallSensorUtility.getInstance().catapultIsTripped()
                && BallSensorUtility.getInstance().indexerIsTripped()) {
            m_currentBallStatus = ballStatus.TwoBalls;
            if (m_currentBallStatus != m_lastBallStatus) {
                flashTop();
            }
        } else if (BallSensorUtility.getInstance().catapultIsTripped()
                || BallSensorUtility.getInstance().indexerIsTripped()) {
            m_currentBallStatus = ballStatus.oneBall;
            if (m_lastBallStatus == ballStatus.TwoBalls) {
                retractTop();
            } else if (m_lastBallStatus == ballStatus.noBall) {
                flashBottom();
            }
        } else if (!BallSensorUtility.getInstance().catapultIsTripped()
                && !BallSensorUtility.getInstance().indexerIsTripped()) {
            m_currentBallStatus = ballStatus.noBall;
            if (m_lastBallStatus == ballStatus.oneBall) {
                retractBottom();
            }
        }
    }

    

    private void endgameState() {
        movingSegmentPattern();
    }

    //----- ENUMS -----\\

    /**
     * <h3>LEDStates</h3>
     */
    public static enum LEDStates {
        Disabled, Autonomous, Teleoperated, Endgame;
    }
}
