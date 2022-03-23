package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BallSensorUtility;


import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDCommand extends CommandBase {

    // ----- CONSTANT(S) -----\\
    private static final int SLOW_TIMER = 32;
    // Used in FlashingLEDPatterm
    private static final int MED_TIMER = 15;
    // Used in ShooterPattern
    private static final int FAST_TIMER = 1;
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
    private static final Color8Bit yellow = new Color8Bit(55, 40, 0);
    private static final Color8Bit red = new Color8Bit(125, 0, 0);
    private static final Color8Bit blue = new Color8Bit(0, 0, 125);
    // ----- VARIABLE(S) -----\\
    // Flag to determine time delays
    // One execute() cycle is 0.020 seconds
    private int counter = 0;
    // Flag to determine State change
    private boolean animCheck = false;
    private int lastBallStatus = 0;
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

    // ------CONSTUCTOR(S)--------\\
    public LEDCommand(LEDSubsystem subsystem, LEDPatterns pattern) {
        m_pattern = pattern;
        m_LEDSubsystem = subsystem;
        m_fullBuffer = m_LEDSubsystem.getBuffer();
        // Rounds up to the next integer
        m_singleStrandBuffer = new AddressableLEDBuffer((int) Math.ceil(m_fullBuffer.getLength() / 4.0));
        m_OffBuffer = createClearStrip();
        m_YellowBuffer = createSolidYellowLEDs();
        solidYellowLEDs();
        addRequirements(m_LEDSubsystem);
    }

    @Override
    public void initialize() {
        // Wait to determine alliance for FMS signal
        // Alliance can change during simulation
        allianceColor = DriverStation.getAlliance();
        counter = 0;
        animCheck = false;
        lastBallStatus = 0;
        //clearStrip();
        switch (m_pattern) {
            case AutonPattern:
                solidAllianceLEDs();
                break;
            case TeleopIdle:
                ballStatusInit();
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
                ballStatus();
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
     * <h3>solidYellowLEDs</h3>
     * Sets LEDs to yellow.
     * Pattern for disabled robot.
     */
    public void solidYellowLEDs() {
        m_LEDSubsystem.setBuffer(m_YellowBuffer);
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
     * <h3>calibrationTest</h3>
     * Test if LED lengths are properly accounted for.
     */
    private void calibrationTest() {
        for (int i = 0; i < m_singleStrandBuffer.getLength(); i++) {
            m_singleStrandBuffer.setLED(i, black); // yellow
        }
        m_singleStrandBuffer.setLED(0, red);
        m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() - 1, red);
        applySingleBuffer();
        for (int i = 0; i < 4; i++) {
            if (m_fullBuffer.getLED8Bit(STRAND_STARTS[i]).equals(red)) {
                m_fullBuffer.setLED(STRAND_STARTS[i], white);
            } else {
                m_fullBuffer.setLED(STRAND_STARTS[i], blue);
            }
            if (m_fullBuffer.getLED8Bit(STRAND_STARTS[i + 1] - 1).equals(red)) {
                m_fullBuffer.setLED(STRAND_STARTS[i + 1] - 1, white);
            } else {
                m_fullBuffer.setLED(STRAND_STARTS[i + 1] - 1, blue);
            }
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
        if (counter >= m_singleStrandBuffer.getLength() * FAST_TIMER) {
            counter = 0;
        }
        // Keeping track of animation speed.
        if (counter % FAST_TIMER == 0) {
            // Writes the length of the strip
            for (int j = 0; j < SEGMENT_LENGTH; j++) {
                // Places overflow to the beginning
                if (j + (counter / FAST_TIMER) < m_singleStrandBuffer.getLength()) {
                    m_singleStrandBuffer.setLED(j + (counter / FAST_TIMER), white);// white
                } else {
                    m_singleStrandBuffer.setLED((j + (counter / FAST_TIMER) - m_singleStrandBuffer.getLength()),
                            white);// white
                }
            }
            // sets the led behind the segment back to off
            if ((counter / FAST_TIMER) > 0) {
                m_singleStrandBuffer.setLED((counter / FAST_TIMER) - 1, black);// off
            }
            applySingleBuffer();
        }
    }

    /**
     * <h3>ballStatusInit</h3>
     * 
     * Sets the starting pattern based on current number of balls.
     */
    private void ballStatusInit() {
        if (BallSensorUtility.getInstance().intakeIsTripped()
                && BallSensorUtility.getInstance().loadedIsTripped()) {
            lastBallStatus = 2;
            solidAllianceLEDs();
        } else if (BallSensorUtility.getInstance().intakeIsTripped()
                || BallSensorUtility.getInstance().loadedIsTripped()) {
            lastBallStatus = 1;
            halfSolidAllianceLEDs();
        } else {
            lastBallStatus = 0;
            clearStrip();
        }
    }

    /**
     * <h3>ballStatus</h3>
     * manages active pattern based off ball sensors
     */
    private void ballStatus() {
        if (BallSensorUtility.getInstance().intakeIsTripped()
                && BallSensorUtility.getInstance().loadedIsTripped()) {
            if (lastBallStatus != 2) {
                flashLEDHighPattern();
            }
        } else if (BallSensorUtility.getInstance().intakeIsTripped()
                || BallSensorUtility.getInstance().loadedIsTripped()) {
            if (lastBallStatus == 2) {
                retractTopLEDs();
            } else if (lastBallStatus == 0) {
                flashLEDLowPattern();
            }
        } else if (!BallSensorUtility.getInstance().intakeIsTripped()
                && !BallSensorUtility.getInstance().loadedIsTripped()) {
            if (lastBallStatus != 0) {
                retractBottomLEDs();
            }
        }
    }

    /**
     * <h3>halfSolidAllianceLEDs</h3>
     * Actives the bottom half of the strip to the alliance color, leaving the rest
     * off.
     */
    private void halfSolidAllianceLEDs() {
        for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
            m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red);
        }
        for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
            m_singleStrandBuffer.setLED(i, black);
        }
        applySingleBuffer();
    }

    /**
     * <h3>flashLEDHighPattern</h3>
     * Flashes alliance color on the top half three times then remains on
     */
    private void flashLEDHighPattern() {
        counter++;
        if (counter > MED_TIMER * 3) {
            lastBallStatus = 2;
            counter = 0;
        } else {
            if (counter % (MED_TIMER) == 0) {
                for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
                    m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }
            } else if (counter % (MED_TIMER) == MED_TIMER/2) {
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
        if (counter > MED_TIMER * 6) {
            lastBallStatus = 1;
            counter = 0; // alliance
        } else {
            if (counter % (MED_TIMER * 2) == 0) {
                for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
                    m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }
            } else if (counter % (MED_TIMER * 2) == MED_TIMER) {
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
        counter++;
        if (counter >= FAST_TIMER * m_singleStrandBuffer.getLength() / 2) {
            lastBallStatus = 1;
            counter = 0;
        } else {
            if (counter % FAST_TIMER == 0) {
                m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() - (counter / FAST_TIMER + 1), black);
                applySingleBuffer();
            }
        }
    }

    /**
     * <h3>retractBottomLEDs</h3>
     * retracts LEDs one by one from half to empty
     */
    private void retractBottomLEDs() {
        if (counter >= FAST_TIMER * m_singleStrandBuffer.getLength() / 2) {
            lastBallStatus = 0;
            counter = 0;
        } else {
            if (counter % FAST_TIMER == 0) {
                m_singleStrandBuffer.setLED(m_singleStrandBuffer.getLength() / 2 - (counter / FAST_TIMER), black);
                applySingleBuffer();
            }
            counter++;
        }
    }

    // =========================IGNORE ALL OTHER PATTERNS-WILL BE REFACTORED
    // LATER==================\
    /**
     * <h3>everyOtherOn</h3>
     * alternating between alliance color and off
     */
    private void everyOtherOff() {
        counter++;
        if (counter >= SLOW_TIMER) {
            if (animCheck == true) {

                /**
                 * sets the LED pattern to:
                 * 10101010101010......
                 * 
                 * (0 being off, 1 being blue)
                 */
                for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                    if (i % 2 == 0) { // Sets every other LED to bright blue
                        m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red
                                                                                                       // based on
                                                                                                       // alliance
                    } else {
                        m_fullBuffer.setLED(i, black);
                    }
                }

                // Sets animCheck to false for pattern switch next time the delay is finished
                animCheck = false;

            } else {

                /**
                 * Clears the LED strip and sets all LED's to off
                 */
                for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                    m_fullBuffer.setLED(i, black);
                }

                /**
                 * Sets the LED pattern to:
                 * 01010101010101......
                 * 
                 * (0 being off, 1 being blue)
                 */
                for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to bright blue
                    m_fullBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based on
                                                                                           // alliance
                }

                // Sets animCheck to true for pattern switch next time the delay is finished
                animCheck = true;

            }
            m_LEDSubsystem.setBuffer(m_fullBuffer);
            // Resets the counter to 0 once the pattern is set
            counter = 0;
        }
    }

    /**
     * <h3>everyOtherOff</h3>
     * alternating between alliance color and white
     */
    private void everyOtherOn() {
        counter++;
        if (counter >= SLOW_TIMER) {
            if (animCheck == true) {

                /**
                 * sets the LED pattern to:
                 * 10101010101010......
                 * 
                 * (0 being off, 1 being blue)
                 */
                for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                    if (i % 2 == 0) { // Sets every other LED to bright blue
                        m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red
                                                                                                       // based on
                                                                                                       // alliance
                    } else {
                        m_fullBuffer.setLED(i, white);
                    }
                }

                // Sets animCheck to false for pattern switch next time the delay is finished
                animCheck = false;

            } else {

                /**
                 * Clears the LED strip and sets all LED's to off
                 */
                for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                    m_fullBuffer.setLED(i, white);
                }

                /**
                 * Sets the LED pattern to:
                 * 01010101010101......
                 * 
                 * (0 being off, 1 being blue)
                 */
                for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                // bright blue
                    m_singleStrandBuffer.setLED(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }

                // Sets animCheck to true for pattern switch next time the delay is finished
                animCheck = true;

            }
            m_LEDSubsystem.setBuffer(m_fullBuffer);
            // Resets the counter to 0 once the pattern is set
            counter = 0;
        }
    }

    /**
     * <h3>blueYellowAlt</h3>
     * alternating between yellow and blue
     */
    private void blueYellowAlt() {

        counter++;

        // If counter is equal or greater to LIGHT_DELAY_NUM_OF_ITERATIONS (basically
        // acting as a delay)
        if (counter >= SLOW_TIMER) {
            animCheck = !animCheck;
            counter = 0;
        }
        if (animCheck == true) {
            /**
             * sets the LED pattern to:
             * 10101010101010......
             * 
             * (0 being off, 1 being yellow)
             */
            for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i += 2) {
                m_fullBuffer.setLED(i, yellow); // Yellow
                m_fullBuffer.setLED(i + 1, blue); // Blue
            }
        } else {
            /*
             * sets the LED pattern to:
             * 01010101010101......
             * 
             * (0 being off, 1 being yellow)
             */
            for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) {
                m_fullBuffer.setLED(i, yellow); // Yellow
                m_fullBuffer.setLED(i - 1, blue); // Blue
            }
        }
        m_LEDSubsystem.setBuffer(m_fullBuffer);
    }

    public static enum LEDPatterns {
        AutonPattern, TeleopIdle, EndgamePatten;

        private LEDPatterns() {
        }
    }
} // End of LEDCommand