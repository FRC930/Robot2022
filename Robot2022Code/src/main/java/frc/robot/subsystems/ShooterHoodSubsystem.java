/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>ShooterHoodSubsystem</h3>
 * Subsystem class to manage the shooter's hood.
 */
public class ShooterHoodSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\
    // Clicks of the TalonFX encoder per rotation of motor shaft
    private static final double TALON_CPR = 2048.0;
    // Gear ratio from motor to the hood(if the hood were a full circle)
    private static final double GEAR_RATIO = (16.0 / 36.0) * (15.0 / 235.0);
    // Maxiumum travel of hood in degrees
    private static final double HOOD_MAX_POSITION = 30.0;
    // PID values
    private static final double MOTOR_KP = 1;
    private static final double MOTOR_KD = 15;

    // -------- DECLARATIONS --------\\
    private final WPI_TalonFX hoodMotor;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>ShooterHoodSubsystem</h3>
     * Creates a subsystem class to manage the shooter hood.
     * 
     * @param hoodMotorID ID of the shooter hood motor
     */
    public ShooterHoodSubsystem(int hoodMotorID) {

        // Motor declaration
        hoodMotor = new WPI_TalonFX(hoodMotorID);

        // Configuration for all of the talon motors set to config obj
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set PID values
        config.slot0.kP = MOTOR_KP;
        config.slot0.kD = MOTOR_KD;

        // Allow encoder deadband to prevent oscillation
        config.slot0.allowableClosedloopError = 25;

        // Set current and voltage control for brake mode
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 30, 30, 0.0);
        config.statorCurrLimit.enable = true;
        config.voltageCompSaturation = 9.0;

        // Sets encoder limits so the hood can't break itself by going too far
        config.forwardSoftLimitThreshold = ((HOOD_MAX_POSITION / 360.0) * TALON_CPR / GEAR_RATIO);
        config.reverseSoftLimitThreshold = 250;
        config.forwardSoftLimitEnable = true;
        config.reverseSoftLimitEnable = true;

        // Sets motor so it can't be manually moved when not being used
        hoodMotor.setNeutralMode(NeutralMode.Brake);

        // Motor is not inverted
        hoodMotor.setInverted(InvertType.None);

        //Repeats sending the config until successful
        ErrorCode error = ErrorCode.CAN_INVALID_PARAM;
        do {
            error = hoodMotor.configAllSettings(config, 500);
            System.out.println("Trying!");
        } while (error != ErrorCode.OK);
    }

    /**
     * <h3>setHoodPosition</h3>
     * Sets the desired position of the shooter hood
     * 
     * @param angle desired hood position in degrees
     */
    public void setHoodPosition(double angle) {
        // Clamps position to the limits
        if (angle < 0) {
            angle = 0;
        } else if (angle > HOOD_MAX_POSITION) {
            angle = HOOD_MAX_POSITION;
        }
        // Converts degrees into encoder ticks
        hoodMotor.set(ControlMode.Position, (angle / 360.0) * TALON_CPR / GEAR_RATIO,
                DemandType.ArbitraryFeedForward, 0.06);
    }

    /**
     * <h3>getHoodPosition</h3>
     * Gets current position of the shooter hood
     * 
     * @return current position of hood in degrees
     */
    public double getHoodPosition() {
        // Converts encoder ticks to degrees of the hood
        return (hoodMotor.getSelectedSensorPosition() / TALON_CPR) * GEAR_RATIO * 360.0;
    }
}
// end of class ShooterHoodSubsystem