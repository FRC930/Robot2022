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

// import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//-------- SUBSYSTEM CLASS --------\\
/**
 * <h3> EndgameMotorSubsystem </h3>
 * 
 * Initializes and controls the two endgame arm motors.
 */
public class EndgameMotorSubsystem extends SubsystemBase {

    private static final double GEAR_RATIO = 100;
    private static final double TALON_CPR = 2048;
    private static final double MOTOR_KP = 0.03;

    //-------- VARIABLES --------\\
    /**
     * The motor controller that controls the endgame motor
     */
    private final WPI_TalonFX endgameMotorMaster;
    private final WPI_TalonFX endgameMotorSlave;

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
        endgameMotorMaster = new WPI_TalonFX(motorIDMaster);
        endgameMotorSlave = new WPI_TalonFX(motorIDSlave);
        // Sets default for added motors
        endgameMotorMaster.configFactoryDefault();
        endgameMotorSlave.configFactoryDefault();
        endgameMotorMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        endgameMotorMaster.config_kP(0, MOTOR_KP);
        // Makes it so it can't be manually moved when neutral
        endgameMotorMaster.setNeutralMode(NeutralMode.Brake);
        endgameMotorSlave.setNeutralMode(NeutralMode.Brake);
        refollowEndgameMotors();
    }

    public void refollowEndgameMotors() {
        // Sets slave to follow master and inverts slave
        endgameMotorMaster.setInverted(InvertType.None);
        endgameMotorSlave.follow(endgameMotorMaster, FollowerType.PercentOutput);
        endgameMotorSlave.setInverted(InvertType.OpposeMaster);
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
        // ControlMode.PercentOutput makes the value be given to the motor as a percent (Ex. 0.90 is 90%)
        endgameMotorMaster.set(ControlMode.PercentOutput, speed);
    }

    /**
     * <h3>stopMotor</h3>
     * 
     * This method makes the endgame motor stop
     */
    public void stopMotor() {
        endgameMotorMaster.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * <h3>getMotorSpeed</h3>
     * This method returns the endgame motor speed
     * 
     * @return the current motor speed asa percent
     */
    public double getMotorSpeed() {
        return endgameMotorMaster.getMotorOutputPercent();
    }

    // TODO: ATTATCH ENCODER AND FINISH GETTER

    /**
     * <h3>getArmRotation</h3>
     * How far the arm has rotated from zero.
     * 
     * @return the position of the encoder 
     */
    public double getArmRotation() {
        // Converts the value to usable units
        System.out.print(endgameMotorMaster.getSelectedSensorPosition() / TALON_CPR / GEAR_RATIO);
        return endgameMotorMaster.getSelectedSensorPosition() / TALON_CPR / GEAR_RATIO;
    }

    /**
     * <h3>setArmPosition</h3>
     * 
     * @param position the wanted position of the arm as a decimal of a rotation.
     */
    public void setArmPosition(double position){
        // Converts back to decimal
        endgameMotorMaster.set(ControlMode.Position, position * GEAR_RATIO * TALON_CPR);
    }
    
    /**
     * <h3>resetEncoderPosition</h3>
     * 
     * Resets the endcoder back to starting position. Should be set while the bar is horizontal.
     */
    public void resetEncoderPosition() {
        endgameMotorMaster.setSelectedSensorPosition(0.0);
    }

} // End of class EndgameMotorSubsystem