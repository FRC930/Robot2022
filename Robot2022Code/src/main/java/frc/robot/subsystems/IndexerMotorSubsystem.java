//----- IMPORTS -----\\

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//----- CLASS -----\\
/**
 * <h3>IndexerMotorSubsystem</h3>
 * 
 * Manages the indexer motors.
 */
public class IndexerMotorSubsystem extends SubsystemBase {

    // ----- MOTORS -----\\

    private final WPI_TalonFX m_stagedIndexer;
    private final WPI_TalonFX m_loadedIndexer;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>IndexerMotorSubsystem</h3>
     * 
     * Manages the indexer motors.
     * 
     * @param stagedID - Can ID of the staged motor
     * @param loadedID - Can ID of the loaded motor
     */
    public IndexerMotorSubsystem(int stagedID, int loadedID) {
        m_stagedIndexer = new WPI_TalonFX(stagedID);
        m_stagedIndexer.stopMotor();
        m_loadedIndexer = new WPI_TalonFX(loadedID);
        m_loadedIndexer.stopMotor();
    }

    // ----- METHODS -----\\

    /**
     * <h3>setStagedMotorSpeed</h3>
     * 
     * Sets the staged indexer motor speed.
     * 
     * @param speed the speed to set the motor
     */
    public void setStagedMotorSpeed(double speed) {
        m_stagedIndexer.set(speed);
    }

    /**
     * <h3>setLoadedMotorSpeed</h3>
     * 
     * Sets the loaded indexer motor speed.
     * 
     * @param speed the speed to set the motor
     */
    public void setLoadedMotorSpeed(double speed) {
        m_loadedIndexer.set(-speed);
    }

    /**
     * <h3>stopMotors</h3>
     * 
     * Stops both indexer motors.
     */
    public void stopMotors() {
        m_stagedIndexer.stopMotor();
        m_loadedIndexer.stopMotor();
        // Need to set voltages to zero because stop motor doesn't actually stop them?
        m_stagedIndexer.setVoltage(0.0);
        m_loadedIndexer.setVoltage(0.0);
    }
}
