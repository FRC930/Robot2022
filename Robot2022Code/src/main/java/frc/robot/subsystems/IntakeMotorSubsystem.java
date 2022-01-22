//-------- IMPORTS --------\\

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h3>IntakeMotorSubsystem</h3>
 * 
 * This class controls the intake motors
 */
public class IntakeMotorSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\

    // -------- DECLARATIONS --------\\

    /**
     * The motor controller that controls the intake motor
     */
    private WPI_TalonSRX intakeMotorController;

    // -------- CONSTRUCTOR --------\

    /**
     * This constructor initializes the {@link #intakeMotorController} to the proper
     * hardware
     */
    public IntakeMotorSubsystem(int intakeID) {
        intakeMotorController = new WPI_TalonSRX(intakeID);
    }

    // -------- METHODS --------\\

    /**
     * <h3>setMotorSpeed</h3>
     * 
     * This method sets the intake motor speed to the passed variable
     * 
     * @param speed the speed at which to set the motor
     */
    public void setMotorSpeed(double speed) {
        intakeMotorController.set(ControlMode.PercentOutput, -speed);
    }

    public WPI_TalonSRX getIntakeMotor() {
        return intakeMotorController;
    }

    /**
     * <h3>getMotorSpeed</h3>
     * This method returns the intake motor speed
     * 
     * @return the current motor speed
     */
    public double getMotorSpeed() {
        return intakeMotorController.getMotorOutputPercent();
    }

} // end of class IntakeMotorSubsystem