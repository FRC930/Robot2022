//----- IMPORTS-----\\

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//----- CLASS -----\\
/**
 * <h3>IntakeMotorSubsystem</h3>
 * 
 * This class controls the intake motors
 */
public class IntakeMotorSubsystem extends SubsystemBase {

    //----- VARIABLES -----\\

    /**
     * The motor controller that controls the intake motor
     */
    private final WPI_TalonFX m_intakeMotorController;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>IntakeMotorSubsystem</h3>
     * 
     * This constructor initializes the {@link #m_intakeMotorController} to the proper
     * hardware
     * 
     * @param intakeID  - Can ID for the intake motor
     */
    public IntakeMotorSubsystem(int intakeID) {
        m_intakeMotorController = new WPI_TalonFX(intakeID);
    }

    //----- METHODS -----\\

    /**
     * <h3>setMotorSpeed</h3>
     * 
     * This method sets the intake motor speed to the passed variable
     * 
     * @param speed the speed at which to set the motor
     */
    public void setMotorSpeed(double speed) {

        //PercentOutput is the amount of output that the motor produces between -1 and 1
        m_intakeMotorController.set(ControlMode.PercentOutput, speed);
    }

    /**
     * <h3>getMotorSpeed</h3>
     * This method returns the intake motor speed
     * 
     * @return the current motor speed
     */
    public double getMotorSpeed() {
        return m_intakeMotorController.getMotorOutputPercent();
    }
} // end of class IntakeMotorSubsystem