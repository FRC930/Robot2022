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
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>ShooterHoodSubsystem</h3>
 * Subsystem class to manage the shooter's hood.
 */
public class ShooterHoodSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\
    // Clicks of the TalonFX encoder per rotation
    private static final double TALON_CPR = 2048.0;
    // TODO: determine hood units
    // Gear ratio from motor to the hood rack
    private static final double GEAR_RATIO = (16.0 / 36.0) * (15.0 / 235.0);
    // Maxiumum travel of hood in fraction of full hood rack rotation
    private static final double HOOD_MAX_POSITION = 30.0 / 360.0;
    // PID for adjustments
    private static final double MOTOR_KP = 0.03;

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

        // Reset configuration
        hoodMotor.configFactoryDefault();

        // Config integrated sensor
        hoodMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        // Set PID values
        hoodMotor.config_kP(0, MOTOR_KP);

        // Set current for brake mode
        hoodMotor.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(true, 5, 6, 0.15));

        // Set encoder limits
        hoodMotor.configForwardSoftLimitThreshold(HOOD_MAX_POSITION * TALON_CPR / GEAR_RATIO, 0);
        hoodMotor.configReverseSoftLimitThreshold(0, 0);
        hoodMotor.configForwardSoftLimitEnable(true, 0);
        hoodMotor.configReverseSoftLimitEnable(true, 0);

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
    public void setHoodPosition(double pos) {
        if (pos < 0) {
            pos = 0;
        } else if (pos > HOOD_MAX_POSITION) {
            pos = HOOD_MAX_POSITION;
        }
        hoodMotor.set(ControlMode.Position, pos * TALON_CPR / GEAR_RATIO);
    }

    public void stopHood() {
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setSlowSpeed() {
        hoodMotor.set(ControlMode.PercentOutput, 0.1);
    }

    public void setSlowRevSpeed() {
        hoodMotor.set(ControlMode.PercentOutput, -0.1);
    }

    /**
     * <h3>getHoodPosition</h3>
     * Gets current position of the shooter hood
     * 
     * @return current position in fraction of hood rotation
     */
    public double getHoodPosition() {
        return (hoodMotor.getSelectedSensorPosition() / TALON_CPR) * GEAR_RATIO;
    }
}
// end of class ShooterHoodSubsystem