package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private Solenoid solenoid1;
    private Solenoid solenoid2;

    /**
     * <h3>CatapultSubsystem</h3>
     * 
     * Initializes a new catapult subsystem with the passed solenoid IDs
     * 
     * @param solenoidID1 ID of the first solenoid
     * @param solenoidID2 ID of the second solenoid
     */
    public CatapultSubsystem(int solenoidID1, int solenoidID2) {
        solenoid1 = new Solenoid(PneumaticsModuleType.REVPH, solenoidID1);
        solenoid2 = new Solenoid(PneumaticsModuleType.REVPH, solenoidID2);

        solenoid1.set(false);
        solenoid2.set(false);
    }

    /**
     * <h3>extend</h3>
     * 
     * Extends the pistons on the catapult
     */
    public void extend() {
        solenoid1.set(true);
        solenoid2.set(true);
    }

    /**
     * <h3>retract</h3>
     * 
     * Retracts the pistons on the catapult
     */
    public void retract() {
        solenoid1.set(false);
        solenoid2.set(false);
    }
}
