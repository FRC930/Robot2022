/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>ShooterHoodSubsystem</h3>
 * Subsystem class to manage the shooter's hood.
 */
public class ShooterHoodSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\
    private static final double TALON_CPR = 2048;
    private static final double GEAR_RATIO = 1;
    private static final double HOOD_MAX_POSITION = 0.3;

    // -------- DECLARATIONS --------\\

    // motor controller for the shooter hood
    private final WPI_TalonFX hoodMotor;

    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>ShooterHoodSubsystem</h3>
     * Creates a subsystem class to manage the shooter's hood.
     * 
     * @param hoodMotorID ID of the shooter's hood's motor
     */
    public ShooterHoodSubsystem(int hoodMotorID) {

        // Motor declaration
        hoodMotor = new WPI_TalonFX(hoodMotorID);
        hoodMotor.configFactoryDefault();
        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        // Sets motor so it can't be manually moved when neutral
        hoodMotor.setNeutralMode(NeutralMode.Brake);

        // Motor is not inverted
        hoodMotor.setInverted(InvertType.None);
    }

    /**
     * <h3>setHoodPosition</h3>
     * Sets the desired position of the shooter hood
     * 
     * @param pos desired position in fraction of hood rotation
     */
    public void setHoodPosition (double pos) {
        if (pos < 0) {
            pos = 0;
        } else if (pos > HOOD_MAX_POSITION) {
            pos = HOOD_MAX_POSITION;
        }
        hoodMotor.set(ControlMode.Position, pos * GEAR_RATIO * TALON_CPR);
    }

    /**
     * <h3>getHoodPosition</h3>
     * Gets current position of the shooter hood
     * 
     * @return current position in fraction of hood rotation
     */
    public double getHoodPosition () {
        return (hoodMotor.getSelectedSensorPosition() / TALON_CPR) / GEAR_RATIO;
    }
}
// end of class ShooterHoodSubsystem