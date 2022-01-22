package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    private final WPI_TalonSRX endgameMotorMaster;
    private final WPI_TalonSRX endgameMotorSlave;

    // -------- CONSTRUCTOR --------\

    /**
     * This constructor initializes the endgame motors to the proper hardware
     * 
     * @param motorIDMaster ID for the master TalonSRX-has endgame encoder attached
     * @param motorIDSlave ID for the slave TalonSRX-has pigeon attached
     */
    public EndgameMotorSubsystem(int motorIDMaster, int motorIDSlave) {
        endgameMotorMaster = new WPI_TalonSRX(motorIDMaster);
        endgameMotorSlave = new WPI_TalonSRX(motorIDSlave);
        endgameMotorSlave.follow(endgameMotorMaster);
        endgameMotorSlave.setInverted(true);
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
    }

    public void stopMotor() {
        endgameMotorMaster.set(ControlMode.PercentOutput, 0.0);
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

    /**
     * Returns slave TalonSRX controller
     * </p>
     * {@link frc.robot.subsystems.DriveSubsystem DriveSubsystem}
     * needs the slave talon for the Pigeon onboard.
     * 
     * @return endgameSlaveMotor
     */
    public WPI_TalonSRX getEndgameMotorSlave() {
        return endgameMotorSlave;
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
     * public double getEncoderPosition(double position) {
     * 
     * }
     */
}
