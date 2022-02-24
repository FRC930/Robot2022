//-------- IMPORTS --------\\

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.GyroUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h3>IntakeMotorSubsystem</h3>
 * 
 * This class controls the intake motors
 */
public class IntakeMotorSubsystem extends SubsystemBase {

    //-------- DECLARATIONS --------\\

    /**
     * The motor controller that controls the intake motor
     */
    private WPI_TalonSRX intakeMotorController;
    // private WPI_TalonFX intakeMotorController;
    // private WPI_TalonSRX gyroController;

    // -------- CONSTRUCTOR --------\

    /**
     * This constructor initializes the {@link #intakeMotorController} to the proper
     * hardware
     */
    public IntakeMotorSubsystem(int intakeID) {
        intakeMotorController = new WPI_TalonSRX(intakeID);
        // intakeMotorController = new WPI_TalonFX(intakeID);
        // gyroController = new WPI_TalonSRX(intakeID);
        GyroUtility.getInstance().setGyro(this);
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

    /**
     * <h3>getMotorSpeed</h3>
     * This method returns the intake motor speed
     * 
     * @return the current motor speed
     */
    public double getMotorSpeed() {
        return intakeMotorController.getMotorOutputPercent();
    }

    /**
     * Returns TalonSRX controller
     * </p>
     * {@link frc.robot.subsystems.DriveSubsystem DriveSubsystem}
     * needs the talon for the Pigeon onboard.
     * 
     * @return intakeMotor
     */
    public WPI_TalonSRX getIntakeMotor() {
        return intakeMotorController;
        // return gyroController;
    }

    public void periodic(){
        BallSensorUtility.getInstance().indexerIsTripped();
    }

} // end of class IntakeMotorSubsystem