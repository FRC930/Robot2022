package frc.robot.utilities;

import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroUtility {
    private static GyroUtility instance;
    private static PigeonIMU m_gyro; 


    private GyroUtility() {

    }
    /**
    * <h3>setGyro</h3>
    * Sets the gyro
    */
    
    /**
    * <h3>getGyro</h3
    * Gets the gyro
     */

    public void setGyro(IntakeMotorSubsystem eSubsystem){
        m_gyro = new PigeonIMU(eSubsystem.getIntakeMotor());
    }
    public PigeonIMU getGyro(){
        // if m_gyro not set return error
        if (m_gyro == null){
            throw new RuntimeException("Tried to get gyro but not yet initialized. Make sure" +
            " intake subsystem in insantiated before drive subsystem.");
        }
        return m_gyro;
    }
    /**
     * <h3>getInstance</h3>
     * 
     * Gyro is a singleton, so getInstance returns the instance of
     * the class that the program will use
     * 
     * @return the instance
     */
    public static GyroUtility getInstance() {
        if (instance == null) {
            instance = new GyroUtility();
        }
        return instance;
    }

}