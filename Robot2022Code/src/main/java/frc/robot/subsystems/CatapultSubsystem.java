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
    //This constant is the delay needed between openBallHolder() and extend()
    public static final double BALL_HOLDER_DELAY = 0.25;

    private Solenoid launchSolenoid1;
    private Solenoid launchSolenoid2;
    private Solenoid launchSolenoid3;
    private Solenoid launchSolenoid4;

    private Solenoid ballHolderSolenoid;

    /**
     * <h3>CatapultSubsystem</h3>
     * 
     * Initializes a new catapult subsystem with the passed solenoid IDs
     * 
     * @param solenoidID1          ID of the first launch solenoid
     * @param solenoidID2          ID of the second launch solenoid
     * @param solenoidID3          ID of the third launch solenoid
     * @param solenoidID4          ID of the fourth launch solenoid
     * @param ballHolderSolenoidID ID of the ball holder's solenoid
     */
    public CatapultSubsystem(int solenoidID1, int solenoidID2, int solenoidID3, int solenoidID4,
            int ballHolderSolenoidID) {
        launchSolenoid1 = new Solenoid(
                Robot.isReal() || solenoidID1 > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                solenoidID1);
        launchSolenoid2 = new Solenoid(
                Robot.isReal() || solenoidID2 > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                solenoidID2);
        launchSolenoid3 = new Solenoid(
                Robot.isReal() || solenoidID3 > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                solenoidID3);
        launchSolenoid4 = new Solenoid(
                Robot.isReal() || solenoidID4 > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                solenoidID4);
        ballHolderSolenoid = new Solenoid(
                Robot.isReal() || ballHolderSolenoidID > 7 ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM,
                ballHolderSolenoidID);

        launchSolenoid1.set(false);
        launchSolenoid2.set(false);
        launchSolenoid3.set(false);
        launchSolenoid4.set(false);

        ballHolderSolenoid.set(false);
    }

    /**
     * <h3>extend</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extend() {
        launchSolenoid1.set(true);
        launchSolenoid2.set(true);
        launchSolenoid3.set(true);
        launchSolenoid4.set(true);
    }

    /**
     * <h3>retract</h3>
     * 
     * Retracts the pistons on the catapult
     */
    public void retract() {
        launchSolenoid1.set(false);
        launchSolenoid2.set(false);
        launchSolenoid3.set(false);
        launchSolenoid4.set(false);
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
