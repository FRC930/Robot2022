package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h3> EndgameMotorSubsystem </h3>
 * 
 * Initializes and controls the two endgame arm motors.
 */
public class EndgameMotorSubsystem extends SubsystemBase {

    // -------- DECLARATIONS --------\\

    /**
     * The motor controller that controls the endgame motor
     */
    private final WPI_TalonFX endgameMotorMaster;
    private final WPI_TalonFX endgameMotorSlave;

    // -------- CONSTRUCTOR --------\

    /**
     * This constructor initializes the endgame motors to the proper hardware
     * 
     * @param motorIDMaster ID for the master TalonFX
     * @param motorIDSlave ID for the slave TalonFX
     */
    public EndgameMotorSubsystem(int motorIDMaster, int motorIDSlave) {
        endgameMotorMaster = new WPI_TalonFX(motorIDMaster);
        endgameMotorSlave = new WPI_TalonFX(motorIDSlave);
        endgameMotorMaster.setNeutralMode(NeutralMode.Brake);
        endgameMotorSlave.setNeutralMode(NeutralMode.Brake);
    }

    // -------- METHODS --------\\

    /**
     * <h3>setMotorSpeed</h3>
     * 
     * This method sets the endgame motor speed to the passed variable
     * 
     * @param speed the speed at which to set the motor
     */
    public void setMotorSpeed(double speed) {
        endgameMotorMaster.set(ControlMode.PercentOutput, speed);
        endgameMotorSlave.set(ControlMode.PercentOutput, -speed);
    }

    public void stopMotor() {
        endgameMotorMaster.set(ControlMode.PercentOutput, 0.0);
        endgameMotorSlave.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * <h3>getMotorSpeed</h3>
     * This method returns the endgame motor speed
     * 
     * @return the current motor speed
     */
    public double getMotorSpeed() {
        return endgameMotorMaster.getMotorOutputPercent();
    }

    // TODO: ATTATCH ENCODER AND FINISH GETTER

    /**
     * getEncoderPosition </p>
     * How far the encoder has rotated from the starting position.
     * 4096 units per rotation
     * 
     * @return the position of the encoder 
     */
    public double getEncoderPosition() {
        System.out.print(endgameMotorMaster.getSelectedSensorPosition());
        return endgameMotorMaster.getSelectedSensorPosition();
    }

    /**
     * resetEncoderPosition </p>
     * Resets the endcoder back to starting position. Should be set while the bar is horizontal.
     */
    public void resetEncoderPosition() {
        endgameMotorMaster.setSelectedSensorPosition(0.0);
    }
    
}