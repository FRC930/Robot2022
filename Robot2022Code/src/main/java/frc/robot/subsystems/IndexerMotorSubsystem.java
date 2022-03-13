//----- IMPORTS -----\\

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//----- CLASS -----\\
/**
 * <h3>IndexerMotorSubsystem</h3>
 * 
 * Manages the indexer motor.
 */
public class IndexerMotorSubsystem extends SubsystemBase {

    // ----- MOTORS -----\\

    private final WPI_TalonFX m_intakeIndexer;
    private final WPI_TalonFX m_loadedIndexer;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>IndexerMotorSubsystem</h3>
     * 
     * Manages the indexer motors.
     * 
     * @param intakeID - Can ID of the intake indexer motor.
     * @param loadedID - Can ID of the loaded indexer motor.
     */
    public IndexerMotorSubsystem(int intakeID, int loadedID) {
        m_intakeIndexer = new WPI_TalonFX(intakeID);
        m_intakeIndexer.stopMotor();
        m_loadedIndexer = new WPI_TalonFX(loadedID);
        m_loadedIndexer.stopMotor();
    }

    // ----- METHODS -----\\

    /**
     * <h3>setIntakeMotorSpeed</h3>
     * 
     * Sets the intake indexer motor speed.
     * 
     * @param speed the speed to set the motor
     */
    public void setIntakeMotorSpeed(double speed) {
        m_intakeIndexer.set(speed);
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
}
