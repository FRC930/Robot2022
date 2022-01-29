package frc.robot.utilities;

import frc.robot.subsystems.EndgameMotorSubsystem;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Gyro {
    private static Gyro instance;
    private static PigeonIMU m_gyro;


    private Gyro() {

    }
    /**
    * <h3>setGyro</h3>
    * Sets the gyro
    */
    
    /**
    * <h3>getGyro</h3
    * Gets the gyro
     */

    public void setGyro(EndgameMotorSubsystem eSubsystem){
        m_gyro = new PigeonIMU(eSubsystem.getEndgameMotorSlave());
    }
    public PigeonIMU getGyro(){
        //  TO-DO if m_gyro not set return error
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
    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro();
        }
        return instance;
    }

}