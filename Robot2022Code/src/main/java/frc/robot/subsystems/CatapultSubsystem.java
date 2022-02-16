package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
     * @param solenoidID1 ID of the first solenoid
     * @param solenoidID2 ID of the second solenoid
     */
    public CatapultSubsystem(int solenoidID1, int solenoidID2, int solenoidID3, int solenoidID4,
            int ballHolderSolenoidID) {
        launchSolenoid1 = new Solenoid(PneumaticsModuleType.REVPH, solenoidID1);
        launchSolenoid2 = new Solenoid(PneumaticsModuleType.REVPH, solenoidID2);
        launchSolenoid3 = new Solenoid(PneumaticsModuleType.REVPH, solenoidID3);
        launchSolenoid4 = new Solenoid(PneumaticsModuleType.REVPH, solenoidID4);

        ballHolderSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ballHolderSolenoidID);

        launchSolenoid1.set(false);
        launchSolenoid2.set(false);
        launchSolenoid3.set(false);
        launchSolenoid4.set(false);

        ballHolderSolenoid.set(true);
    }

    /**
     * <h3>extend</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extend() {
        ballHolderSolenoid.set(false);

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

        ballHolderSolenoid.set(true);
    }

    public void periodic(){
        BallSensorUtility.getInstance().catapultIsTripped();
    }
}
