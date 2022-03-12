package frc.robot.utilities;

import com.ctre.phoenix.sensors.Pigeon2;

public class GyroUtility {
    private static GyroUtility instance;
    private Pigeon2 m_gyro;

    private GyroUtility() {
        m_gyro = new Pigeon2(9);
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

    /**
     * <h3>getGyro</h3
     * Gets the gyro
     */
    public Pigeon2 getGyro() {
        return m_gyro;
    }
}