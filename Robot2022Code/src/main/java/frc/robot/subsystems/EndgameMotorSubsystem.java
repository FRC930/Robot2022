package frc.robot.subsystems;

// import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class EndgameMotorSubsystem {

     //-------- DECLARATIONS --------\\

    /**
     * The motor controller that controls the endgame motor
     */
    private WPI_TalonSRX endgameMotorLeft; 
    private WPI_TalonSRX endgameMotorRight;

    //-------- CONSTRUCTOR --------\

    /**
     * This constructor initializes the {@link #endgameMotorController} to the proper hardware
     */
    public EndgameMotorSubsystem(int motorIDLeft, int motorIDRight) {
        endgameMotorLeft = new WPI_TalonSRX(motorIDLeft);
        endgameMotorRight = new WPI_TalonSRX(motorIDRight);
        endgameMotorLeft.follow(endgameMotorRight);
    }
    
    //-------- METHODS --------\\
    
    /**
     * <h3>setMotorSpeed</h3>
     * 
     * This method sets the endgame motor speed to the passed variable
     * 
     * @param speed the speed at which to set the motor
     */
    public void setMotorSpeed(double speed) {
        endgameMotorLeft.set(ControlMode.PercentOutput, speed);
    }

    public void stopMotor() {
        endgameMotorLeft.set(ControlMode.PercentOutput, 0.0);
 
    }

     /**
     * <h3>getMotorSpeed</h3>
     * This method returns the endgame motor speed
     * 
     * @return the current motor speed
     */
    public double getMotorSpeed() {
        return endgameMotorLeft.getMotorOutputPercent();  
    }

    /**
     * <h3>getEncoderPosition</h3>
     * How far the motor has rotated from it's original position
     * Units are 2048 per rotation
     * 
     * @param position the position at which to set the encoder
     * @return the current encoder positio
     */
    // TODO: ATTATCH ENCODER AND FINISH GETTER
    /*
    public double getEncoderPosition(double position) {
        
    }*/
}
