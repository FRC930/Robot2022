//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

//----- CLASS -----\\
/**
 * <h3>CatapultSubsystem</h3>
 * 
 * Interfaces with the robot to provide access to the pistons controlling the
 * catapult
 */
public class CatapultSubsystem extends SubsystemBase {

    //----- CONSTANTS -----\\

    // This constant is the delay when firing between openBallHolder() and extend()
    public static final double CATAPULT_FIRE_DELAY = 0.50;
    // This constant is the time of pulse for the launch solenoids
    private static final double CATAPULT_PULSE_DURATION = 0.5;

    public static final double SHOOT_TIMEOUT = 0.25;

    //----- SOLENOIDS -----\\

    private final Solenoid m_launchSolenoidLarge1;
    private final Solenoid m_launchSolenoidLarge2;
    private final Solenoid m_launchSolenoidSmall1;
    private final Solenoid m_launchSolenoidSmall2;

    private final Solenoid m_ballHolderSolenoid;
    private final Solenoid m_shotControlSolenoid;
    private final Solenoid m_retractor;

    /**
     * <h3>CatapultSubsystem</h3>
     * 
     * Initializes a new catapult subsystem with the passed solenoid IDs
     * 
     * @param frontLeftID   ID of the first launch solenoid
     * @param frontRightID  ID of the second launch solenoid
     * @param rearLeftID    ID of the third launch solenoid
     * @param rearRightID   ID of the fourth launch solenoid
     * @param ballHolderID  ID of the ball holder solenoid
     * @param shotControlID ID of the shot control solenoid
     * @param retractorID   ID of the retractor solenoid
     */
    public CatapultSubsystem(int frontLeftID, int frontRightID, int rearLeftID, int rearRightID,
            int ballHolderID, int shotControlID, int retractorID) {
        m_launchSolenoidLarge1 = new Solenoid(
                Robot.isReal() || frontLeftID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                frontLeftID);
        m_launchSolenoidSmall1 = new Solenoid(
                Robot.isReal() || frontRightID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                frontRightID);
        m_launchSolenoidSmall2 = new Solenoid(
                Robot.isReal() || rearLeftID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                rearLeftID);
        m_launchSolenoidLarge2 = new Solenoid(
                Robot.isReal() || rearRightID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                rearRightID);
        m_ballHolderSolenoid = new Solenoid(
                Robot.isReal() || ballHolderID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                ballHolderID);
        m_shotControlSolenoid = new Solenoid(
                Robot.isReal() || shotControlID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                shotControlID);
        m_retractor = new Solenoid(
                Robot.isReal() || retractorID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                retractorID);

        m_launchSolenoidLarge1.setPulseDuration(CATAPULT_PULSE_DURATION);
        m_launchSolenoidLarge2.setPulseDuration(CATAPULT_PULSE_DURATION);
        m_launchSolenoidSmall1.setPulseDuration(CATAPULT_PULSE_DURATION);
        m_launchSolenoidSmall2.setPulseDuration(CATAPULT_PULSE_DURATION);
        m_retractor.setPulseDuration(0.05);

        m_launchSolenoidLarge1.set(false);
        m_launchSolenoidLarge2.set(false);
        m_launchSolenoidSmall1.set(false);
        m_launchSolenoidSmall2.set(false);
        m_shotControlSolenoid.set(false);
        m_ballHolderSolenoid.set(false);
        m_retractor.set(false);
    }

    /**
     * <h3>extendLargePistons</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extendLargePistons() {
        m_launchSolenoidLarge1.set(true);
        m_launchSolenoidLarge2.set(true);
    }

    /**
     * <h3>extendSmallPistons</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extendSmallPistons() {
        m_launchSolenoidSmall1.set(true);
        m_launchSolenoidSmall2.set(true);
    }

    /**
     * <h3>extendAllPistons</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extendAllPistons() {
        m_launchSolenoidLarge1.set(true);
        m_launchSolenoidLarge2.set(true);
        m_launchSolenoidSmall1.set(true);
        m_launchSolenoidSmall2.set(true);
    }

    /**
     * <h3>retractAll</h3>
     * 
     * Retracts all pistons on the catapult
     */
    public void retractAll() {
        m_launchSolenoidLarge1.set(false);
        m_launchSolenoidLarge2.set(false);
        m_launchSolenoidSmall1.set(false);
        m_launchSolenoidSmall2.set(false);
        m_retractor.startPulse();
    }

    /**
     * <h3>retractRectractor</h3>
     * 
     * Retracts the retraction piston.
     */
    public void retractRetractor() {
        m_retractor.startPulse();
    }

    /**
     * <h3>setLongShot</h3>
     * 
     * Angles the catapult launch trajectory for long distances.
     */
    public void setLongShot() {
        m_shotControlSolenoid.set(true);
    }

    /**
     * <h3>setShortShot</h3>
     * 
     * Angles the catapult launch trajectory for short distances.
     */
    public void setShortShot() {
        m_shotControlSolenoid.set(false);
    }

    /**
     * <h3>closeBallHolder</h3>
     *
     * Closes the ball holder to keep ball in place
     */
    public void closeBallHolder() {
        m_ballHolderSolenoid.set(true);
    }

    /**
     * <h3>openBallHolder</h3>
     *
     * Opens the ball holder to let ball be shot and let new one in
     */
    public void openBallHolder() {
        m_ballHolderSolenoid.set(false);
    }

    /**
     * <h3>ballHolderIsClosed</h3>
     *
     * @return if the ballHolder is currently closed
     */
    public boolean ballHolderIsClosed() {
        return m_ballHolderSolenoid.get();
    }
}
