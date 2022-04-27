//----- IMPORTS -----\\

package frc.robot.commands;

import frc.robot.ControllerManager;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;

//----- CLASS -----\\
/**
 * <h3>LEDCommand</h3>
 * 
 * Manages all LED patterns and sends patterns to the LEDSubsystem of where it is literally deployed to the LED strip.
 */
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
    //private static final int STRAND_STARTS[] = { 0, 75, 150, 225, 300 };
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
    private enum BallStatus {
        oneBall, TwoBalls, noBall
    };

    // manages the last ball status
    private BallStatus m_lastBallStatus;
    // manages the current ball statue
    private BallStatus m_currentBallStatus;
    // current Alliance color
    private Alliance allianceColor;
    // The LED Subsystem (strip) itself
    private final LEDSubsystem m_LEDSubsystem;
    // Selected LED pattern from enum
    private final LEDPatterns m_pattern;

    /*
    BUFFERS
    Buffers are a virtual placeholder that represents each of LEDs on the LED strip.
     */
    // Compile data is sent to the LED strip through the subsystem
    private final AddressableLEDBuffer m_fullBuffer;
    private final int m_SingleSideLength;
    // Precompiled buffers for solid patterns
    private final AddressableLEDBuffer m_OffBuffer;
    private final AddressableLEDBuffer m_YellowBuffer;
    private final AddressableLEDBuffer m_GreenBuffer;

    private final ControllerManager m_driverController;

    private int beamStartPosition;
    private int beamEndPosition;

    private boolean aimIsPressed;

    // ------CONSTUCTOR(S)--------\\
    /**
     * <h3>LEDCommand</h3>
     * 
     * Manages all LED patterns and sends patterns to the LEDSubsystem of where it is literally deployed to the LED strip.
     * 
     * @param subsystem         LEDSubsystem to send LED buffers to
     * @param driverController  Driver controller for teleoperated patterns
     * @param pattern           Patterns to use
     */
    public LEDCommand(LEDSubsystem subsystem, ControllerManager driverController, LEDPatterns pattern) {
        m_pattern = pattern;
        m_LEDSubsystem = subsystem;

        // Gets the full sized buffer from the LEDSubsystem
        m_fullBuffer = m_LEDSubsystem.getBuffer();

        m_driverController = driverController;

        // Gets the length of a single side of the 4 sided LED strip
        // Rounds up to the next integer
        m_SingleSideLength = (int) Math.ceil(m_fullBuffer.getLength() / 4.0);

        // Creates buffers for off, disabled, and aimed to avoid using excessive for loops
        m_OffBuffer = createClearStrip();
        m_YellowBuffer = createSolidYellowLEDs();
        m_GreenBuffer = createSolidGreenLEDs();

        // Sets the LEDs to yellow to show the robot is in a disabled state
        solidYellowLEDs();

        // Sets the number of balls currently held to no balls
        m_lastBallStatus = BallStatus.noBall;
        m_currentBallStatus = m_lastBallStatus;

        // The aim button is currently not pressed on the driver controller
        aimIsPressed = false;

        addRequirements(m_LEDSubsystem);

    }

    //----- METHODS -----\\

    /**
     * <h3>initialize</h3>
     * 
     * Runs when the command is first ran in the CommandScheduler.
     * Sets default values and which pattern to be used on command initialization.
     */
    @Override
    public void initialize() {
        // Wait to determine alliance for FMS signal
        // Alliance can change during simulation
        allianceColor = DriverStation.getAlliance();
        counter = 0;
        beamStartPosition = -SEGMENT_LENGTH;
        beamEndPosition = 0;

        // Determines which pattern to send to the LED strip on command initialization
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

    /**
     * <h3>execute</h3>
     * 
     * Runs while the command is being run by the CommandScheduler.
     * Determines what pattern to run while the command is active.
     */
    @Override
    public void execute() {
        // Determines which pattern to run while the command is active.
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

    /**
     * <h3>isFinished</h3>
     * 
     * Runs when the command is finished running in the CommandScheduler.
     * 
     * Due to the command being intended to be used in default, return false.
     * 
     * @return false
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * <h3>applyBuffer</h3>
     * 
     * Applies the single Buffer to the full buffer 4 times.
     */
    private void applyLEDValue(int initialPos, Color8Bit color) {
            //  Right Rear LEDs run from positions (0 - 74)
            m_fullBuffer.setLED(initialPos, color);
            //  Right Front LEDs run from positions (149 - 75)
            m_fullBuffer.setLED(((m_SingleSideLength*2) - 1) - initialPos, color);
            //  Left Front LEDs run from positions (150 - 224)
            m_fullBuffer.setLED((m_SingleSideLength*2) + initialPos, color);
            //  Left Rear LEDs run from positions (299 - 225)
            m_fullBuffer.setLED(((m_SingleSideLength*4) - 1) - initialPos, color);
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
     * 
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
     * 
     * Shifts a segment of LEDs along the strip.
     * pattern for Endgame
     */
    private void movingSegmentPattern() {
        // Increments the counter every 20 ms
        counter++;

        // If the counter is greater than or equal to a single side multiplied by the endgame timer, set it to 0.
        // Prevents counter from exceeding max size.
        if (counter >= m_SingleSideLength * ENDGAME_TIMER) {
            counter = 0;
        }

        // Keeping track of animation speed.
        if (counter % ENDGAME_TIMER == 0) {

            // If the end of the beam is greater than or equal to the length of a single side, wrap to the bottom of the side.
            if (beamEndPosition >= m_SingleSideLength) {
                beamEndPosition = 0;
            }

            // If the start of the beam is greater than or equal to the length of a single side, wrap to the bottom of the side.
            if (beamStartPosition >= m_SingleSideLength) {
                beamStartPosition = 0;
            }

            // If the beam start position is greater than or equal to 0, turn every LED it encounters off.
            if (beamStartPosition >= 0) {
                applyLEDValue(beamStartPosition, black);
            }

            // Turn every LED that the beam end position encounters on to the alliance color.
            applyLEDValue(beamEndPosition, (allianceColor == Alliance.Blue) ? blue : red);

            // Increments the start and end positions every 20 ms.
            beamStartPosition += 1;
            beamEndPosition += 1;

            // Deploys the buffer to the LEDSubsystem
            m_LEDSubsystem.setBuffer(m_fullBuffer);
        }
    }

    /**
     * <h3>ballStatus</h3>
     * 
     * Manages active pattern based off ball sensors.
     */
    private void teleopStatus() {
        // If the left bumper on the driver controller is pressed and aimStatus returns true, set the last ball status to null and aimIsPressed to true.
        if (m_driverController.getLeftBumper().get() && aimStatus()){
            m_lastBallStatus = BallStatus.noBall;
            aimIsPressed = true;
        
        // If the aim is pressed clear the strip and set aimIsPressed to false
        } else if(aimIsPressed) {
            clearStrip();
            aimIsPressed = false;

        // If the intake and loaded shooter sensors are tripped, set the ball status to two
        } else if (
            BallSensorUtility.getInstance().intakeIsTripped()
            && BallSensorUtility.getInstance().loadedIsTripped()
        ) {
            m_currentBallStatus = BallStatus.TwoBalls;

            // Flashes the top half of the LEDs if there weren't previously 2 balls
            if (m_currentBallStatus != m_lastBallStatus) {
                flashLEDHighPattern();
            }

        // If either the intake or loaded shooter sensors are tripped, set the ball status to one
        } else if (
            BallSensorUtility.getInstance().intakeIsTripped()
            || BallSensorUtility.getInstance().loadedIsTripped()
        ) {
            m_currentBallStatus = BallStatus.oneBall;

            // If the robot previously had two balls, retract the top half
            if (m_lastBallStatus == BallStatus.TwoBalls) {
                retractTopLEDs();

            // If the robot had no balls, flash the bottom half
            } else if (m_lastBallStatus == BallStatus.noBall) {
                flashLEDLowPattern();
            }
        
        // If neither the intake or loaded shooter sensors are tripped, set the ball status to null
        } else if (
            !BallSensorUtility.getInstance().intakeIsTripped()
            && !BallSensorUtility.getInstance().loadedIsTripped()
        ) {
            m_currentBallStatus = BallStatus.noBall;

            // If the robot previously had one ball, retract the bottom half
            if (m_lastBallStatus == BallStatus.oneBall) {
                retractBottomLEDs();
            }
        }
    }

    /**
     * <h3>flashLEDHighPattern</h3>
     * 
     * Flashes alliance color on the top half three times then remains on
     */
    private void flashLEDHighPattern() {
        // Increments the counter every 20 ms
        counter++;

        // If the counter is greater than the flash timer * 3, set the ball status to two and counter to 0
        if (counter > FLASH_TIMER * 3) {
            m_lastBallStatus = BallStatus.TwoBalls;
            counter = 0;

        // Else flash the top LEDs 3 times
        } else {

            // If the counter has no remainder when divided by FLASH_TIMER, turn the LEDs on to the alliance color
            if (counter % (FLASH_TIMER) == 0) {
                for (int i = m_SingleSideLength / 2; i < m_SingleSideLength; i++) {
                    applyLEDValue(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }

            // If the counter has a remainder when divided by FLASH_TIMER that equals FLASH_TIMER / 2, turn the LEDs off
            } else if (counter % (FLASH_TIMER) == FLASH_TIMER / 2) {
                for (int i = m_SingleSideLength / 2; i < m_SingleSideLength; i++) {
                   applyLEDValue(i, black); // off
                }
            }
        }

        // Deploys the buffer to the LEDSubsystem
        m_LEDSubsystem.setBuffer(m_fullBuffer);
    }

    /**
     * <h3>flashLEDLowPattern</h3>
     * 
     * Flashes alliance color on the bottom half three times then remains on
     */
    private void flashLEDLowPattern() {
        // Increments the counter every 20 ms
        counter++;

        // If the counter is greater than the flash timer * 6, set the ball status to one and counter to 0
        if (counter > FLASH_TIMER * 6) {
            m_lastBallStatus = BallStatus.oneBall;
            counter = 0;

        // Else flash the top LEDs 3 times
        } else {

            // If the counter has no remainder when divided by FLASH_TIMER, turn the LEDs on to the alliance color
            if (counter % (FLASH_TIMER * 2) == 0) {
                for (int i = 0; i < m_SingleSideLength / 2; i++) {
                    applyLEDValue(i, (allianceColor == Alliance.Blue) ? blue : red); // blue or red based
                                                                                                   // on alliance
                }

            // If the counter has a remainder when divided by FLASH_TIMER that equals FLASH_TIMER / 2, turn the LEDs off
            } else if (counter % (FLASH_TIMER * 2) == FLASH_TIMER) {
                for (int i = 0; i < m_SingleSideLength / 2; i++) {
                    applyLEDValue(i, black); // off
                }
            }
        }

        // Deploys the buffer to the LEDSubsystem
        m_LEDSubsystem.setBuffer(m_fullBuffer);
    }

    /**
     * <h3>retractTopLEDs</h3>
     * 
     * Retracts LEDs one by one from full to half
     */
    private void retractTopLEDs() {
        // If the counter is greater than or equal to ENDGAME_TIMER * a single side length / 2, set the ball status to one and counter to 0
        if (counter >= ENDGAME_TIMER * m_SingleSideLength / 2) {
            m_lastBallStatus = BallStatus.oneBall;
            counter = 0;
        
        // Else turns of LEDs one by one from the top to the half.
        } else {
            if (counter % ENDGAME_TIMER == 0) {
                applyLEDValue(m_SingleSideLength - (counter / ENDGAME_TIMER + 1), black);
                
                // Deploys the buffer to the LEDSubsystem
                m_LEDSubsystem.setBuffer(m_fullBuffer);
            }
            counter++;
        }
    }

    /**
     * <h3>retractBottomLEDs</h3>
     * 
     * Retracts LEDs one by one from half to empty
     */
    private void retractBottomLEDs() {
        // If the counter is greater than or equal to ENDGAME_TIMER * a single side length / 2, set the ball status to none and counter to 0
        if (counter >= ENDGAME_TIMER * m_SingleSideLength / 2) {
            m_lastBallStatus = BallStatus.noBall;
            counter = 0;
        } else {

            // If the counter has a remainder when divided by FLASH_TIMER that equals FLASH_TIMER / 2, turn the LEDs off
            if (counter % ENDGAME_TIMER == 0) {
                applyLEDValue(m_SingleSideLength / 2 - (counter / ENDGAME_TIMER), black);
                
                //
                //  set buffer
                m_LEDSubsystem.setBuffer(m_fullBuffer);
            }
            counter++;
        }
    }

    /**
     * <h3>aimStatus</h3>
     * 
     * Returns a boolean with a value based on if the robot is aimed or not.
     */
    private boolean aimStatus() {
        // Gets the boolean value of the AIMED datapoint from the Shuffleboard on whether or not the robot is aimed
        // If aimed, set LEDs to green, if not, clear the strip
        if ((boolean) ShuffleboardUtility.getInstance().getFromShuffleboard(ShuffleboardKeys.AIMED).getData()) {
            solidGreenLEDs();
            return true;
        } else {
            clearStrip();
            return false;
        }
    }

    /**
     * <h3>LEDPatterns</h3>
     * 
     * Determines which patterns to use.
     */
    public static enum LEDPatterns {
        AutonPattern, TeleopIdle, EndgamePatten;
    }
} // End of LEDCommand