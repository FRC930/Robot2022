package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.BallSensorUtility;

/**
 * <h3>CatapultSubsystem</h3>
 * 
 * Interfaces with the robot to provide access to the pistons controlling the
 * catapult
 * 
 * @author <a href="https://github.com/Jelombo">Jelombo</a>,
 *         <a href="https://github.com/awtpi314">awtpi314</a>
 * @since 01/20/2022
 * @version 1.0.0
 */
public class CatapultSubsystem extends SubsystemBase {
    // This constant is the delay when firing between openBallHolder() and extend()
    public static final double CATAPULT_FIRE_DELAY = 0.75;
    // This constant is the time of pulse for the launch solenoids
    private final double CATAPULT_PULSE_DURATION = 0.5;

    private Solenoid launchSolenoidLarge1;
    private Solenoid launchSolenoidLarge2;
    private Solenoid launchSolenoidSmall1;
    private Solenoid launchSolenoidSmall2;

    private Solenoid ballHolderSolenoid;

    /**
     * <h3>CatapultSubsystem</h3>
     * 
     * Initializes a new catapult subsystem with the passed solenoid IDs
     * 
     * @param frontLeftID  ID of the first launch solenoid
     * @param frontRightID ID of the second launch solenoid
     * @param rearLeftID   ID of the third launch solenoid
     * @param rearRightID  ID of the fourth launch solenoid
     * @param ballHolderID ID of the ball holder solenoid
     */
    public CatapultSubsystem(int frontLeftID, int frontRightID, int rearLeftID, int rearRightID,
            int ballHolderID) {
        launchSolenoidLarge1 = new Solenoid(
                Robot.isReal() || frontLeftID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                frontLeftID);
        launchSolenoidSmall1 = new Solenoid(
                Robot.isReal() || frontRightID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                frontRightID);
        launchSolenoidSmall2 = new Solenoid(
                Robot.isReal() || rearLeftID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                rearLeftID);
        launchSolenoidLarge2 = new Solenoid(
                Robot.isReal() || rearRightID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                rearRightID);
        ballHolderSolenoid = new Solenoid(
                Robot.isReal() || ballHolderID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                ballHolderID);

        launchSolenoidLarge1.setPulseDuration(CATAPULT_PULSE_DURATION);
        launchSolenoidLarge2.setPulseDuration(CATAPULT_PULSE_DURATION);
        launchSolenoidSmall1.setPulseDuration(CATAPULT_PULSE_DURATION);
        launchSolenoidSmall2.setPulseDuration(CATAPULT_PULSE_DURATION);
        
        launchSolenoidLarge1.set(false);
        launchSolenoidLarge2.set(false);
        launchSolenoidSmall1.set(false);
        launchSolenoidSmall2.set(false);
        ballHolderSolenoid.set(false);
    }

    /**
     * <h3>extendLargePistons</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extendLargePistons() {
        launchSolenoidLarge1.startPulse();
        launchSolenoidLarge2.startPulse();
    }

    /**
     * <h3>extendSmallPistons</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extendSmallPistons() {
        launchSolenoidSmall1.startPulse();
        launchSolenoidSmall2.startPulse();
    }

    /**
     * <h3>extendAllPistons</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extendAllPistons() {
        launchSolenoidLarge1.startPulse();
        launchSolenoidLarge2.startPulse();
        launchSolenoidSmall1.startPulse();
        launchSolenoidSmall2.startPulse();
    }

    /**
     * <h3>retract</h3>
     * 
     * Retracts the pistons on the catapult
     */
    public void retract() {
        launchSolenoidLarge1.set(false);
        launchSolenoidLarge2.set(false);
        launchSolenoidSmall1.set(false);
        launchSolenoidSmall2.set(false);
    }

    /**
     * <h3>closeBallHolder</h3>
     *
     * Closes the ball holder to keep ball in place
     */
    public void closeBallHolder() {
        ballHolderSolenoid.set(true);
    }

    /**
     * <h3>openBallHolder</h3>
     *
     * Opens the ball holder to let ball be shot and let new one in
     */
    public void openBallHolder() {
        ballHolderSolenoid.set(false);
    }

    // Needs to be run for the shuffleboard
    public void periodic() {
        BallSensorUtility.getInstance().catapultIsTripped();
    }
}
