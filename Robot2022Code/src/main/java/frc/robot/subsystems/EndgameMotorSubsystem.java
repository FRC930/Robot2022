/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h3> EndgameMotorSubsystem </h3>
 * 
 * Initializes and controls the two endgame arm motors.
 */
public class EndgameMotorSubsystem extends SubsystemBase {

    //-------- CONSTANTS --------\\
    private static final double GEAR_RATIO = 125;
    private static final double TALON_CPR = 2048;
    private static final double MOTOR_KP = 0.03;

    //-------- VARIABLES --------\\
    /**
     * The motor controller that controls the endgame motor
     */
    private final WPI_TalonFX m_endgameMotorMaster;
    private final WPI_TalonFX m_endgameMotorSlave;

    //-------- CONSTRUCTOR --------\\

    /**
     * <h3>EndgameMotorSubsystem</h3>
     * 
     * Initializes and controls the two endgame arm motors.
     * 
     * @param motorIDMaster ID for the master TalonFX
     * @param motorIDSlave ID for the slave TalonFX
     */
    public EndgameMotorSubsystem(int motorIDMaster, int motorIDSlave) {

        m_endgameMotorMaster = new WPI_TalonFX(motorIDMaster);
        m_endgameMotorSlave = new WPI_TalonFX(motorIDSlave);

        // Sets default for added motors
        m_endgameMotorMaster.configFactoryDefault();
        m_endgameMotorSlave.configFactoryDefault();
        m_endgameMotorMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        m_endgameMotorMaster.config_kP(0, MOTOR_KP);

        // Sets motors so they can't be manually moved when neutral
        m_endgameMotorMaster.setNeutralMode(NeutralMode.Brake);
        m_endgameMotorSlave.setNeutralMode(NeutralMode.Brake);

        refollowEndgameMotors();
    }

    //-------- METHODS --------\\

    // Needed to overcome stopMotor() calls by CTRE's WPI motor controls
    // See https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/28
    /**
     * <h3>refollowEndgameMotors</h3>
     * 
     * Needed to overcome stopMotor() calls by CTRE's WPI motor controls
     * See https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/28
     */
    public void refollowEndgameMotors() {

        // Sets slave to follow master and inverts slave
        m_endgameMotorMaster.setInverted(InvertType.None);
        m_endgameMotorSlave.follow(m_endgameMotorMaster, FollowerType.PercentOutput);
        m_endgameMotorSlave.setInverted(InvertType.OpposeMaster);
    }

    /**
     * <h3>setMotorSpeed</h3>
     * 
     * This method sets the endgame motor speed to the passed variable
     * 
     * @param speed the speed at which to set the motor
     */
    public void setMotorSpeed(double speed) {

        // ControlMode.PercentOutput makes the value be given to the motor as a percent (Ex. 0.90 is 90%)
        m_endgameMotorMaster.set(ControlMode.PercentOutput, speed);
    }

    /**
     * <h3>stopMotor</h3>
     * 
     * This method makes the endgame motor stop
     */
    public void stopMotor() {
        m_endgameMotorMaster.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * <h3>getMotorSpeed</h3>
     * This method returns the endgame motor speed
     * 
     * @return the current motor speed asa percent
     */
    public double getMotorSpeed() {
        return m_endgameMotorMaster.getMotorOutputPercent();
    }

    /**
     * <h3>getArmRotation</h3>
     * How far the arm has rotated from zero.
     * 
     * @return the position of the encoder 
     */
    public double getArmRotation() {
        // Converts the value to usable units
        return m_endgameMotorMaster.getSelectedSensorPosition() / TALON_CPR / GEAR_RATIO;
    }

    /**
     * <h3>setArmPosition</h3>
     * 
     * @param position the wanted position of the arm as a decimal of a rotation.
     */
    public void setArmPosition(double position){
        // Converts back to decimal
        m_endgameMotorMaster.set(ControlMode.Position, position * GEAR_RATIO * TALON_CPR);
    }
    
    /**
     * <h3>resetEncoderPosition</h3>
     * 
     * Resets the endcoder back to starting position. Should be set while the bar is horizontal.
     */
    public void resetEncoderPosition() {
        m_endgameMotorMaster.setSelectedSensorPosition(0.0);
    }

} // End of class EndgameMotorSubsystem