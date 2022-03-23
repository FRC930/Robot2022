/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ShooterUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>ShooterSubsystem</h3>
 * Subsystem class to manage the shooter wheels.
 */
public class ShooterSubsystem extends SubsystemBase {

    // -------- DECLARATIONS --------\\
    // motor controllers for the shooter wheels
    private final WPI_TalonFX topShooter;
    private final WPI_TalonFX bottomShooterMaster;
    private final WPI_TalonFX bottomShooterFollower;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>ShooterSubsystem</h3>
     * Creates a subsystem class to manage the shooter.
     * 
     * @param bottomShooterMasterID   ID of the bottom motor master of the shooter
     * @param bottomShooterFollowerID ID of the bottom motor follower of the shooter
     * @param topShooterID            ID of the top motor of the shooter
     */
    public ShooterSubsystem(int bottomShooterMasterID,
            int bottomShooterFollowerID, int topShooterID) {

        // Motor declaration
        topShooter = new WPI_TalonFX(topShooterID);
        bottomShooterMaster = new WPI_TalonFX(bottomShooterMasterID);
        bottomShooterFollower = new WPI_TalonFX(bottomShooterFollowerID);

        // Reset motors
        topShooter.configFactoryDefault();
        bottomShooterMaster.configFactoryDefault();
        bottomShooterFollower.configFactoryDefault();

        // Sets motors to coast so that they can move freely when neutral
        topShooter.setNeutralMode(NeutralMode.Coast);
        bottomShooterMaster.setNeutralMode(NeutralMode.Coast);
        bottomShooterFollower.setNeutralMode(NeutralMode.Coast);

        // All motors are not inverted
        topShooter.setInverted(InvertType.None);
        refollowShooterMotors();

        // Need to bring shooter values with 0 to prevent null pointer
        ShooterUtility.setValuesToShuffleboard(0.0);
    }

    /**
     * <h3>refollowShooterMotors</h3>
     * Refollows the bottom shooter motors.
     */
    public void refollowShooterMotors() {
        bottomShooterMaster.setInverted(InvertType.None);
        bottomShooterFollower.follow(bottomShooterMaster, FollowerType.PercentOutput);
        bottomShooterFollower.setInverted(InvertType.FollowMaster);
    }

    // -------- METHODS --------\\
    /**
     * <h3>setBottomSpeed</h3>
     * Sets the bottom shooter motor speed.
     *
     * @param speed speed of the bottom wheel in percent output
     */
    public void setBottomSpeed(double speed) {
        // Sets speed to 0 if speed argument is less than 0
        bottomShooterMaster.set(ControlMode.PercentOutput, Math.max(0.0, speed));
    }

    /**
     * <h3>setTopSpeed</h3>
     * Sets the top shooter motor speed.
     *
     * @param speed speed of the top wheel in percent output
     */
    public void setTopSpeed(double speed) {
        // Sets speed to 0 if speed argument is less than 0
        topShooter.set(ControlMode.PercentOutput, Math.max(0.0, speed));
    }

    /**
     * <h3>stopMotors</h3>
     * Stops both shooter motors.
     */
    public void stopMotors() {
        topShooter.stopMotor();
        bottomShooterMaster.stopMotor();
    }

    /**
     * <h3>getBottomSpeed</h3>
     * Gets the bottom shooter motor speed.
     *
     * @return speed of the bottom wheel in percent output
     */
    public double getBottomSpeed() {
        return bottomShooterMaster.getMotorOutputPercent();
    }

    /**
     * <h3>getTopSpeed</h3>
     * Gets the top shooter motor speed.
     *
     * @return speed of the top wheel in percent output
     */
    public double getTopSpeed() {
        return topShooter.getMotorOutputPercent();
    }
} // end of class ShooterSubsystem