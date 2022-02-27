package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BallSensorUtility;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // Flag to determine time delays
    // One execute() cycle is 0.020 seconds
    private int counter = 0;
    // Flag to determine State change
    private boolean animCheck = false;
    private boolean hadTwoBalls = false;
    private boolean hadOneBall = false;
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

    private final AddressableLEDBuffer m_TurnOffBuffer;
    private final AddressableLEDBuffer m_YellowBuffer;
    private AddressableLEDBuffer m_AllianceBuffer;
    private int lastBallStatus=0;

    // ------CONSTUCTOR(S)--------\\
    public LEDCommand(LEDSubsystem subsystem, LEDPatterns pattern) {
        m_pattern = pattern;
        m_LEDSubsystem = subsystem;
        m_fullBuffer = m_LEDSubsystem.getBuffer();
        m_singleStrandBuffer = new AddressableLEDBuffer(m_fullBuffer.getLength() / 4);
        m_TurnOffBuffer = createClearStrip();
        m_YellowBuffer = createSolidYellowLEDs();
        //m_AllianceBuffer = createSolidAllianceLEDs(); // Need to in Initialize for right color
        addRequirements(m_LEDSubsystem);
        //solidYellowLEDs();
    }

    @Override
    public void initialize() {
        // Wait to determine alliance for FMS signal
        // Alliance can change during simulation
        allianceColor = DriverStation.getAlliance();
        m_AllianceBuffer = createSolidAllianceLEDs(); 
        counter = 0;
        animCheck = false;
        hadTwoBalls = false;
        hadOneBall = false;
        clearStrip();
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

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }


    /**
     * <h3>applyBuffer</h3>
     * applys the single Buffer to the full Buffer 4 times
     */
    private void applyBuffer() {
        for (int i = 0; i < m_fullBuffer.getLength(); i++) {
            if (i % (m_singleStrandBuffer.getLength() * 2) < m_singleStrandBuffer.getLength()) {
                m_fullBuffer.setLED(i, m_singleStrandBuffer.getLED8Bit(i % m_singleStrandBuffer.getLength()));
            } else {
                m_fullBuffer.setLED(
                        ((int)(i / m_singleStrandBuffer.getLength() + 1) * m_singleStrandBuffer.getLength())
                                - (i % m_singleStrandBuffer.getLength() + 1),
                        m_singleStrandBuffer.getLED8Bit(i % m_singleStrandBuffer.getLength()));
            }
        }
        m_LEDSubsystem.setBuffer(m_fullBuffer);
    }

    /**
     * <h3>clearStrip</h3>
     * Turns off all LEDs.
     * @return 
     */
    private AddressableLEDBuffer createClearStrip() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_LEDSubsystem.getBufferLength());
        //buffer = m_LEDSubsystem.getBuffer();
        for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
            buffer.setRGB(i, 0, 0, 0); // off
        }
        return buffer;
    }
    private void clearStrip() {
        m_LEDSubsystem.setBuffer(m_TurnOffBuffer);
    }

    /**
     * <h3>solidYellowLEDs</h3>
     * Sets all LEDs to yellow.
     * Pattern for disabled robot.
     */
    public AddressableLEDBuffer createSolidYellowLEDs() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_LEDSubsystem.getBufferLength());
        for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
            buffer.setRGB(i, 50, 40, 0); // yellow
        }
        return buffer;
    }
    public void solidYellowLEDs() {
        m_LEDSubsystem.setBuffer(m_YellowBuffer);
    }

    /**
     * <h3>solidAllianceLEDs</h3>
     * Sets the LED strip to robot's alliance color.
     * pattern for Autonomous
     */
    public AddressableLEDBuffer createSolidAllianceLEDs() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(m_LEDSubsystem.getBufferLength());
        for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
            if (allianceColor == Alliance.Blue) {
                buffer.setRGB(i, 0, 0, 200); // blue
            } else if (allianceColor == Alliance.Red) {
                buffer.setRGB(i, 200, 0, 0); // red
            }
        }
        return buffer;
    }
    public void solidAllianceLEDs() {
        m_LEDSubsystem.setBuffer(m_AllianceBuffer);
    }

    /**
     * <h3>movingSegmentPattern</h3>
     * Shifts a segment of LEDs along the strip.
     * pattern for Endgame
     */
    public void movingSegmentPattern() {
        // Keeping track of animation speed.
        if (counter >= FAST_TIMER) {
            for (int i = 0; i <= m_singleStrandBuffer.getLength(); i++) {
                // Writes the length of the strip
                for (int j = 0; j < SEGMENT_LENGTH; j++) {
                    // Places overflow to the beginning
                    if (j + i < m_singleStrandBuffer.getLength()) {
                        m_singleStrandBuffer.setRGB(j + i, 150, 150, 150);// white
                    } else {
                        m_singleStrandBuffer.setRGB((j + i - m_singleStrandBuffer.getLength()),
                                150, 150, 150);// white
                    }
                }
                // sets the led behind the segment back to off
                if (i > 0) {
                    m_singleStrandBuffer.setRGB(i - 1, 0, 0, 0);// off
                }
            }
            counter = 0;
            applyBuffer();
        }
        counter++;
    }

    /**
     * <h3>ballStatusInit</h3>
     * 
     */
    public void ballStatusInit() {

    }

    /**
     * <h3>ballStatus</h3>
     * manages active pattern based off ball sensors
     */
    private void ballStatus() {
        ;
        if (BallSensorUtility.getInstance().catapultIsTripped()
                && BallSensorUtility.getInstance().indexerIsTripped()
                && !hadTwoBalls) {
            if(lastBallStatus !=1) {
                flashLEDHighPattern();
                lastBallStatus = 1;    
            }
        } else if (BallSensorUtility.getInstance().catapultIsTripped()
                || BallSensorUtility.getInstance().indexerIsTripped() && !hadOneBall) {
            if(lastBallStatus != 2) {
                flashLEDLowPattern();
                lastBallStatus = 2;
            }
        } else if (BallSensorUtility.getInstance().catapultIsTripped()
                || BallSensorUtility.getInstance().indexerIsTripped() && hadTwoBalls) {
            if(lastBallStatus != 3) {
                retractTopLEDs();
                lastBallStatus = 3;
            }

        } else if (!BallSensorUtility.getInstance().catapultIsTripped()
                && !BallSensorUtility.getInstance().indexerIsTripped() && hadOneBall) {
            if(lastBallStatus != 4) {
                retractBottomLEDs();
                lastBallStatus = 4;
            }
        } else {
            if(lastBallStatus != 5) {
                clearStrip();
                lastBallStatus = 5;
            }
        }
    }

    /**
     * <h3>flashLEDHighPattern</h3>
     * Flashes alliance color on the top half three times then remains on
     */
    private void flashLEDHighPattern() {
        if (counter == MED_TIMER * 6) {
            hadTwoBalls = true;
            hadOneBall = true;
            for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
                if (allianceColor == Alliance.Blue) {
                    m_singleStrandBuffer.setRGB(i, 0, 0, 200); // Blue
                }
                if (allianceColor == Alliance.Red) {
                    m_singleStrandBuffer.setRGB(i, 200, 0, 0); // red
                }
            }
            counter++;
        } else if (counter < MED_TIMER * 6) {
            if (counter % (MED_TIMER * 2) < MED_TIMER) {
                for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
                    m_singleStrandBuffer.setRGB(i, 0, 0, 0); // off
                }
            } else {
                for (int i = m_singleStrandBuffer.getLength() / 2; i < m_singleStrandBuffer.getLength(); i++) {
                    if (allianceColor == Alliance.Blue) {
                        m_singleStrandBuffer.setRGB(i, 0, 0, 200); // Blue
                    } else if (allianceColor == Alliance.Red) {
                        m_singleStrandBuffer.setRGB(i, 200, 0, 0); // red
                    }
                }
            }
            counter++;
        }
        applyBuffer();
    }

    /**
     * <h3>flashLEDLowPattern</h3>
     * Flashes alliance color on the bottom half three times then remains on
     */
    private void flashLEDLowPattern() {
        if (counter == MED_TIMER * 6) {
            hadOneBall = true;
            for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
                if (allianceColor == Alliance.Blue) {
                    m_singleStrandBuffer.setRGB(i, 0, 0, 200); // Blue
                }
                if (allianceColor == Alliance.Red) {
                    m_singleStrandBuffer.setRGB(i, 200, 0, 0); // red
                }
            }
            counter++;
        } else if (counter < MED_TIMER * 6) {
            if (counter % (MED_TIMER * 2) < MED_TIMER) {
                for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
                    m_singleStrandBuffer.setRGB(i, 0, 0, 0); // off
                }
            } else {
                for (int i = 0; i < m_singleStrandBuffer.getLength() / 2; i++) {
                    if (allianceColor == Alliance.Blue) {
                        m_singleStrandBuffer.setRGB(i, 0, 0, 200); // Blue
                    } else if (allianceColor == Alliance.Red) {
                        m_singleStrandBuffer.setRGB(i, 200, 0, 0); //
                    }
                }
            }
            counter++;
        }
        applyBuffer();
    }

    /**
     * <h3>retractTopLEDs</h3>
     * retracts LEDs one by one from full to half
     */
    private void retractTopLEDs() {
        if (counter < MED_TIMER * m_singleStrandBuffer.getLength() / 2) {
            if (counter % MED_TIMER == 0) {
                int i = counter / MED_TIMER;
                if (allianceColor == Alliance.Blue) {
                    m_singleStrandBuffer.setRGB(i, 0, 0, 200); // Blue
                } else if (allianceColor == Alliance.Blue) {
                    m_singleStrandBuffer.setRGB(i, 200, 0, 0); // red
                }
                applyBuffer();
            }
            counter++;
        } else {
            hadTwoBalls = false;
            hadOneBall = true;
        }
    }

    /**
     * <h3>retractBottomLEDs</h3>
     * retracts LEDs one by one from half to empty
     */
    private void retractBottomLEDs() {
        if (counter < MED_TIMER * m_singleStrandBuffer.getLength() / 2) {
            if (counter % MED_TIMER == 0) {
                int i = counter / MED_TIMER;
                if (allianceColor == Alliance.Blue) {
                    m_singleStrandBuffer.setRGB(i, 0, 0, 200); // Blue
                } else if (allianceColor == Alliance.Blue) {
                    m_singleStrandBuffer.setRGB(i, 200, 0, 0); // red
                }
                applyBuffer();
            }
            counter++;
        } else {
            hadTwoBalls = false;
            hadOneBall = false;
        }
    }

    /**
     * <h3>everyOtherOn</h3>
     * alternating between alliance color and off
     */
    private void everyOtherOff() {
        counter++;
        if (counter >= SLOW_TIMER) {

            // BLUE ALLIANCE
            if (allianceColor == Alliance.Blue) {

                if (animCheck == true) {

                    /**
                     * sets the LED pattern to:
                     * 10101010101010......
                     * 
                     * (0 being off, 1 being blue)
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        if (i % 2 == 0) { // Sets every other LED to bright blue
                            m_fullBuffer.setRGB(i, 0, 0, 200);
                        } else {
                            m_fullBuffer.setRGB(i, 0, 0, 0);
                        }
                    }

                    // Sets animCheck to false for pattern switch next time the delay is finished
                    animCheck = false;

                } else {

                    /**
                     * Clears the LED strip and sets all LED's to off
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        m_fullBuffer.setRGB(i, 0, 0, 0);
                    }

                    /**
                     * Sets the LED pattern to:
                     * 01010101010101......
                     * 
                     * (0 being off, 1 being blue)
                     */
                    for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                    // bright blue
                        m_fullBuffer.setRGB(i, 0, 0, 200);
                    }

                    // Sets animCheck to true for pattern switch next time the delay is finished
                    animCheck = true;

                }

            }

            // RED ALLIANCE
            if (allianceColor == Alliance.Red) {

                if (animCheck == true) {

                    /**
                     * sets the LED pattern to:
                     * 10101010101010......
                     * 
                     * (0 being off, 1 being red)
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        if (i % 2 == 0) { // Sets every other LED to bright red
                            m_fullBuffer.setRGB(i, 200, 0, 0);
                        } else {
                            m_fullBuffer.setRGB(i, 0, 0, 0);
                        }
                    }

                    // Sets animCheck to false for pattern switch next time the delay is finished
                    animCheck = false;

                } else {

                    /**
                     * clears the LED strip and sets all LED's to off
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        m_fullBuffer.setRGB(i, 0, 0, 0);
                    }

                    /**
                     * Sets the LED pattern to:
                     * 01010101010101......
                     * 
                     * (0 being off, 1 being red)
                     */
                    for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                    // bright red
                        m_fullBuffer.setRGB(i, 200, 0, 0);
                    }

                    // Sets animCheck to true for pattern switch next time the delay is finished
                    animCheck = true;

                }

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

            // BLUE ALLIANCE
            if (allianceColor == Alliance.Blue) {

                if (animCheck == true) {

                    /**
                     * sets the LED pattern to:
                     * 10101010101010......
                     * 
                     * (0 being off, 1 being blue)
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        if (i % 2 == 0) { // Sets every other LED to bright blue
                            m_fullBuffer.setRGB(i, 0, 0, 200);
                        } else {
                            m_fullBuffer.setRGB(i, 150, 150, 150);
                        }
                    }

                    // Sets animCheck to false for pattern switch next time the delay is finished
                    animCheck = false;

                } else {

                    /**
                     * Clears the LED strip and sets all LED's to off
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        m_fullBuffer.setRGB(i, 150, 150, 150);
                    }

                    /**
                     * Sets the LED pattern to:
                     * 01010101010101......
                     * 
                     * (0 being off, 1 being blue)
                     */
                    for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                    // bright blue
                        m_fullBuffer.setRGB(i, 0, 0, 200);
                    }

                    // Sets animCheck to true for pattern switch next time the delay is finished
                    animCheck = true;

                }

            }

            // RED ALLIANCE
            if (allianceColor == Alliance.Red) {

                if (animCheck == true) {

                    /**
                     * sets the LED pattern to:
                     * 10101010101010......
                     * 
                     * (0 being off, 1 being red)
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        if (i % 2 == 0) { // Sets every other LED to bright red
                            m_fullBuffer.setRGB(i, 200, 0, 0);
                        } else {
                            m_fullBuffer.setRGB(i, 150, 150, 150);
                        }
                    }

                    // Sets animCheck to false for pattern switch next time the delay is finished
                    animCheck = false;

                } else {

                    /**
                     * clears the LED strip and sets all LED's to off
                     */
                    for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                        m_fullBuffer.setRGB(i, 150, 150, 150);
                    }

                    /**
                     * Sets the LED pattern to:
                     * 01010101010101......
                     * 
                     * (0 being off, 1 being red)
                     */
                    for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                        m_fullBuffer.setRGB(i, 200, 0, 0);// bright red
                    }

                    // Sets animCheck to true for pattern switch next time the delay is finished
                    animCheck = true;

                }

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
                m_fullBuffer.setRGB(i, 150, 150, 0); // Yellow
                m_fullBuffer.setRGB(i + 1, 0, 0, 150); // Blue
            }
        } else {
            /*
             * sets the LED pattern to:
             * 01010101010101......
             * 
             * (0 being off, 1 being yellow)
             */
            for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) {
                m_fullBuffer.setRGB(i, 150, 150, 0); // Yellow
                m_fullBuffer.setRGB(i - 1, 0, 0, 150); // Blue
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