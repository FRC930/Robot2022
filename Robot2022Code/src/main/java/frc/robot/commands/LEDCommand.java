package frc.robot.commands;

import frc.robot.ControllerManager;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.AimStatusUtility;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDCommand extends CommandBase {

    // ----- CONSTANT(S) -----\\
    // Used in FlashingLEDPatterm
    private static final int FLASH_TIMER = 5;
    // Used in ShooterPattern
    private static final int ENDGAME_TIMER = 1;
    // used in movingSegmentPattern
    private static final int SEGMENT_LENGTH = 20;
    // Starting index for each strip
    // This is needed due to the uneven distribution of LEDs on the week 1 robot
    // (lost one at the beginning)
    // Order is as follows: Front Left, Back Left, Front Right, Back Right, Total #
    // of indexes
    private static final int STRAND_STARTS[] = { 0, 74, 149, 224, 299 };
    // Color presets
    private static final Color8Bit black = new Color8Bit(0, 0, 0);
    private static final Color8Bit white = new Color8Bit(80, 80, 80);
    private static final Color8Bit red = new Color8Bit(125, 0, 0);
    private static final Color8Bit yellow = new Color8Bit(55, 40, 0);
    private static final Color8Bit green = new Color8Bit(0, 125, 0);
    private static final Color8Bit blue = new Color8Bit(0, 0, 125);

    // ----- VARIABLE(S) -----\\
    // Flag to determine time delays
    // One execute() cycle is 0.020 seconds
    private int counter = 0;

    // Flag to determine State change
    private enum ballStatus {
        oneBall, TwoBalls, noBall
    };

    // manages the last ball status
    private ballStatus m_lastBallStatus;
    // manages the current ball statue
    private ballStatus m_currentBallStatus;
    // current Alliance color
    private Alliance allianceColor;
    // The LED Subsystem (strip) itself
    private final LEDSubsystem m_LEDSubsystem;
    // Selected LED pattern from enum
    private final LEDPatterns m_pattern;
    // Virtual placeholder for the LED strip
    // Compile data is sent to the LED strip through the subsystem
    private final AddressableLEDBuffer m_fullBuffer;
    private final AddressableLEDBuffer m_singleStrandBuffer;
    // Precompiled buffers for solid patterns
    private final AddressableLEDBuffer m_OffBuffer;
    private final AddressableLEDBuffer m_YellowBuffer;
    private final AddressableLEDBuffer m_GreenBuffer;

    private final ControllerManager m_driverController;

    private int beamStartPosition;
    private int beamEndPosition;

    private boolean aimIsPressed;

    // ------CONSTUCTOR(S)--------\\
    public LEDCommand(LEDSubsystem subsystem, ControllerManager driverController, LEDPatterns pattern) {
        m_pattern = pattern;
        m_LEDSubsystem = subsystem;
        m_fullBuffer = m_LEDSubsystem.getBuffer();
        m_driverController = driverController;
        // Rounds up to the next integer
        m_singleStrandBuffer = new AddressableLEDBuffer((int) Math.ceil(m_fullBuffer.getLength() / 4.0));
        m_OffBuffer = createClearStrip();
        m_YellowBuffer = createSolidYellowLEDs();
        m_GreenBuffer = createSolidGreenLEDs();
        solidYellowLEDs();
        m_lastBallStatus = ballStatus.noBall;
        m_currentBallStatus = m_lastBallStatus;

        aimIsPressed = false;
        addRequirements(m_LEDSubsystem);

    }

    @Override
    public void initialize() {
        // Wait to determine alliance for FMS signal
        // Alliance can change during simulation
        allianceColor = DriverStation.getAlliance();
        counter = 0;
        beamStartPosition = -SEGMENT_LENGTH;
        beamEndPosition = 0;
        // clearStrip();
        switch (m_pattern) {
            case AutonPattern:
                solidAllianceLEDs();
                break;
            case TeleopIdle:
                clearStrip();
                teleopStatus();
                break;
            default:
                break;
        }
    }

    @Override
    public void execute() {
        switch (m_pattern) {
            case EndgamePatten:
                movingSegmentPattern();
                break;
            case TeleopIdle:
                teleopStatus();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * <h3>applyBuffer</h3>
     * applys the single Buffer to the full Buffer 4 times
     */
    private void applySingleBuffer() {
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

    /**
     * <h3>createClearStrip</h3>
     * Generates the buffer to turn off all LEDs.
     * 
     * @return the buffer of off LEDs
     */
    private AddressableLEDBuffer createClearStrip() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_LEDSubsystem.getBufferLength());
        // buffer = m_LEDSubsystem.getBuffer();
        for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
            buffer.setLED(i, black); // off
        }
        return buffer;
    }

    /**
     * <h3>clearStrip</h3>
     * Turns off all LEDs.
     */
    private void clearStrip() {
        m_LEDSubsystem.setBuffer(m_OffBuffer);
    }

    /**
     * <h3>createSolidYellowLEDs</h3>
     * 
     * Generates buffer to set LEDs to yellow.
     * Pattern for disabled robot.
     * 
     * @return the buffer of yellow LEDs
     */
    private AddressableLEDBuffer createSolidYellowLEDs() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_fullBuffer.getLength());
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, yellow); // yellow
        }
        return buffer;
    }

    /**
     * <h3>createSolidGreenLEDs</h3>
     * 
     * Generates buffer to set LEDs to green.
     * Pattern for aimed robot.
     * 
     * @return the buffer of green LEDs
     */
    private AddressableLEDBuffer createSolidGreenLEDs() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_fullBuffer.getLength());
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, green);
        }
        return buffer;
    }

    /**
     * <h3>solidYellowLEDs</h3>
     * 
     * Sets LEDs to yellow.
     * Pattern for disabled robot.
     */
    public void solidYellowLEDs() {
        m_LEDSubsystem.setBuffer(m_YellowBuffer);
    }

    /**
     * <h3>solidGreenLEDs</h3>
     * 
     * Sets LEDs to green.
     * Pattern for aimed robot.
     */
    public void solidGreenLEDs() {
        m_LEDSubsystem.setBuffer(m_GreenBuffer);
    }

    /**
     * <h3>solidAllianceLEDs</h3>
     * Sets the LED strip to robot's alliance color.
     * pattern for Autonomous
     */
    private void solidAllianceLEDs() {
        for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
            m_fullBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based on alliance
        }
        m_LEDSubsystem.setBuffer(m_fullBuffer);
    }

    /**
     * <h3>movingSegmentPattern</h3>
     * Shifts a segment of LEDs along the strip.
     * pattern for Endgame
     */
    private void movingSegmentPattern() {
        counter++;
        if (counter >= m_singleStrandBuffer.getLength() * ENDGAME_TIMER) {
            counter = 0;
        }
        // Keeping track of animation speed.
        if (counter % ENDGAME_TIMER == 0) {

            if (beamEndPosition > m_singleStrandBuffer.getLength()) {
                beamEndPosition = 0;
            }
            if (beamStartPosition > m_singleStrandBuffer.getLength()) {
                beamStartPosition = 0;
            }

            if (beamStartPosition >= 0) {
                m_singleStrandBuffer.setLED(beamStartPosition, black);
            }

            m_singleStrandBuffer.setLED(beamEndPosition, (allianceColor == Alliance.Blue) ? blue : red);

            beamStartPosition += 1;
            beamEndPosition += 1;

            applySingleBuffer();
        }
    }

    /**
     * <h3>ballStatus</h3>
     * manages active pattern based off ball sensors
     */
    private void teleopStatus() {
        if (m_driverController.getRightBumper().get() 
                && !m_driverController.getLeftBumper().get() && aimStatus()){
            
            m_lastBallStatus = ballStatus.noBall;
            aimIsPressed = true;
               
        } else if(aimIsPressed) {
            clearStrip();
            aimIsPressed = false;
        } else if (BallSensorUtility.getInstance().catapultIsTripped()
                && BallSensorUtility.getInstance().indexerIsTripped()) {
            m_currentBallStatus = ballStatus.TwoBalls;
            if (m_currentBallStatus != m_lastBallStatus) {
                flashLEDHighPattern();
            }
        } else if (BallSensorUtility.getInstance().catapultIsTripped()
                || BallSensorUtility.getInstance().indexerIsTripped()) {
            m_currentBallStatus = ballStatus.oneBall;
            if (m_lastBallStatus == ballStatus.TwoBalls) {
                retractTopLEDs();
            } else if (m_lastBallStatus == ballStatus.noBall) {
                flashLEDLowPattern();
            }
        } else if (!BallSensorUtility.getInstance().catapultIsTripped()
                && !BallSensorUtility.getInstance().indexerIsTripped()) {
            m_currentBallStatus = ballStatus.noBall;
            if (m_lastBallStatus == ballStatus.oneBall) {
                retractBottomLEDs();
            }
        }
        // if (!BallSensorUtility.getInstance().catapultIsTripped()
        //         && !BallSensorUtility.getInstance().indexerIsTripped()) {
        //     clearStrip();
        // }
    }

    /**
     * <h3>flashLEDHighPattern</h3>
     * Flashes alliance color on the top half three times then remains on
     */
    private void flashLEDHighPattern() {
        counter++;
        if (counter > FLASH_TIMER * 3) {
            m_lastBallStatus = ballStatus.TwoBalls;
            counter = 0;
        } else {
            if (counter % (FLASH_TIMER) == 0) {
                for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
                    m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }
            } else if (counter % (FLASH_TIMER) == FLASH_TIMER / 2) {
                for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
                    m_singleStrandBuffer.setLED(i, black); // off
                }
            }
        }
        applySingleBuffer();
    }

    /**
     * <h3>flashLEDLowPattern</h3>
     * Flashes alliance color on the bottom half three times then remains on
     */
    private void flashLEDLowPattern() {
        counter++;
        if (counter > FLASH_TIMER * 6) {
            m_lastBallStatus = ballStatus.oneBall;
            counter = 0; // alliance
        } else {
            if (counter % (FLASH_TIMER * 2) == 0) {
                for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
                    m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }
            } else if (counter % (FLASH_TIMER * 2) == FLASH_TIMER) {
                for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
                    m_singleStrandBuffer.setLED(i, black); // off
                }
            }
        }
        applySingleBuffer();
    }

    /**
     * <h3>retractTopLEDs</h3>
     * retracts LEDs one by one from full to half
     */
    private void retractTopLEDs() {
        m_lastBallStatus = ballStatus.oneBall;
        counter++;
        if (counter >= ENDGAME_TIMER * m_singleStrandBuffer.getLength() / 2) {
            counter = 0;
        } else {
            if (counter % ENDGAME_TIMER == 0) {
                m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() - (counter / ENDGAME_TIMER + 1), black);
                applySingleBuffer();
            }
        }
    }

    /**
     * <h3>retractBottomLEDs</h3>
     * retracts LEDs one by one from half to empty
     */
    private void retractBottomLEDs() {
        if (counter >= ENDGAME_TIMER * m_singleStrandBuffer.getLength() / 2) {
            m_lastBallStatus = ballStatus.noBall;
            counter = 0;
        } else {
            if (counter % ENDGAME_TIMER == 0) {
                m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() / 2 - (counter / ENDGAME_TIMER), black);
                applySingleBuffer();
            }
            counter++;
        }
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
            solidGreenLEDs();
            return true;
        } else {
            clearStrip();
        }
        return false;
    }

    public static enum LEDPatterns {
        AutonPattern, TeleopIdle, EndgamePatten;
    }

    public static enum LEDStates {
        Disabled, Autonomous, BallStatus, Aimed, Endgame;
    }
} // End of LEDCommand