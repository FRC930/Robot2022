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
 * <h3>FlywheelSubsystem</h3>
 * Subsystem class to manage the flywheel and all related hardware.
 */
public class FlywheelSubsystem extends SubsystemBase {

    // -------- DECLARATIONS --------\\
    // motor controllers for the shooter
    private final WPI_TalonFX topFlywheel;
    private final WPI_TalonFX bottomFlywheelMaster;
    private final WPI_TalonFX bottomFlywheelFollower;

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>FlywheelSubsystem</h3>
     * Creates a subsystem class to manage the flywheel and all related hardware.
     * 
     * @param topFlywheelID            ID of the top motor of the flywheel
     * @param bottomFlywheelMasterID   ID of the bottom motor master of the flywheel
     * @param bottomFlywheelFollowerID ID of the bottom motor of follower the
     *                                 flywheel
     */
    public FlywheelSubsystem(int topFlywheelID, int bottomFlywheelMasterID,
            int bottomFlywheelFollowerID) {

        // Motor declaration
        topFlywheel = new WPI_TalonFX(topFlywheelID);
        bottomFlywheelMaster = new WPI_TalonFX(bottomFlywheelMasterID);
        bottomFlywheelFollower = new WPI_TalonFX(bottomFlywheelFollowerID);

        // Reset motors
        topFlywheel.configFactoryDefault();
        bottomFlywheelMaster.configFactoryDefault();
        bottomFlywheelFollower.configFactoryDefault();

        // Sets motors to coast so that they can move freely when neutral
        topFlywheel.setNeutralMode(NeutralMode.Coast);
        bottomFlywheelMaster.setNeutralMode(NeutralMode.Coast);
        bottomFlywheelFollower.setNeutralMode(NeutralMode.Coast);

        // All motors are not inverted
        topFlywheel.setInverted(InvertType.None);
        refollowShooterMotors();

        // Need to bring shooter values with 0 to prevent null pointer
        ShooterUtility.setValuesToShuffleboard(0.0);
    }

    /**
     * <h3>refollowShooterMotors</h3>
     * Refollows the bottom flywheel motors.
     */
    public void refollowShooterMotors() {
        bottomFlywheelMaster.setInverted(InvertType.None);
        bottomFlywheelFollower.follow(bottomFlywheelMaster, FollowerType.PercentOutput);
        bottomFlywheelFollower.setInverted(InvertType.FollowMaster);
    }

    // -------- METHODS --------\\
    /**
     * <h3>setBottomSpeed</h3>
     * Sets the bottom flywheel motor speed.
     *
     * @param speed speed of the bottom wheel in percent output
     */
    public void setBottomSpeed(double speed) {
        if (speed > 0) {
            bottomFlywheelMaster.set(ControlMode.PercentOutput, speed);
        }
        else{
            bottomFlywheelMaster.set(ControlMode.PercentOutput, 0.0);
        }
    }

    /**
     * <h3>setTopSpeed</h3>
     * Sets the top flywheel motor speed.
     *
     * @param speed speed of the top wheel in percent output
     */
    public void setTopSpeed(double speed) {
        if (speed > 0) {
            topFlywheel.set(ControlMode.PercentOutput, speed);
        }
        else{
            topFlywheel.set(ControlMode.PercentOutput, 0.0);
        }
    }

    /**
     * <h3>getBottomSpeed</h3>
     * Gets the bottom flywheel motor speed.
     *
     * @return speed of the bottom wheel in percent output
     */
    public double getBottomSpeed() {
        return bottomFlywheelMaster.getMotorOutputPercent();
    }

    /**
     * <h3>getTopSpeed</h3>
     * Gets the top flywheel motor speed.
     *
     * @return speed of the top wheel in percent output
     */
    public double getTopSpeed() {
        return topFlywheel.getMotorOutputPercent();
    }
}
// end of class FlywheelSubsystem