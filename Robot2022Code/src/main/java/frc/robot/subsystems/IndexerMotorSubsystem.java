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

    //----- MOTORS -----\\

    private final WPI_TalonFX m_indexerMotor;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>IndexerMotorSubsystem</h3>
     * 
     * Manages the indexer motor.
     * 
     * @param id    - Can ID of the indexer motor.
     */
    public IndexerMotorSubsystem(int id){
        m_indexerMotor = new WPI_TalonFX(id);
        m_indexerMotor.stopMotor();
    }

    //----- METHODS -----\\
    
    /**
     * <h3>setMotorSpeed</h3>
     * 
     * Sets the indexer motor speed.
     * 
     * @param speed the speed to set the motor
     */
    public void setMotorSpeed(double speed) {
        m_indexerMotor.set(-speed);
    }
}
