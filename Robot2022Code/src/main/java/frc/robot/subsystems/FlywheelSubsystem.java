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
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//-------- SUBSYSTEM CLASS --------\\

/**
 * <h3>FlywheelSubsystem</h3>
 * Subsystem class to manage the flywheel and all related hardware.
 */
public class FlywheelSubsystem extends SubsystemBase {

    // -------- CONSTANTS --------\\
    // Clicks of the TalonFX encoder per rotation
    private static final double TALON_CPR = 2048.0;
    // Gear ratio from motor to top wheel
    private static final double TOP_GEAR_RATIO = 30.0 / 36.0;
    // Gear ratio from motor to bottom wheel
    private static final double BOTTOM_GEAR_RATIO = 36.0 / 24.0;
    // PID values for velocity adjustment
    private static final double MOTOR_KF = 0.5;

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

        // Configure integrated sensors
        topFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        bottomFlywheelMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        // Set PID values
        topFlywheel.config_kF(0, MOTOR_KF);
        bottomFlywheelMaster.config_kF(0, MOTOR_KF);

        // Sets motors to coast so that they can move freely when neutral
        topFlywheel.setNeutralMode(NeutralMode.Coast);
        bottomFlywheelMaster.setNeutralMode(NeutralMode.Coast);
        bottomFlywheelFollower.setNeutralMode(NeutralMode.Coast);

        // All motors are not inverted
        topFlywheel.setInverted(InvertType.None);
        refollowShooterMotors();
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
     * @param velocity speed of the bottom wheel in RPMs
     */
    public void setBottomSpeed(double velocity) {
        //TODO: FIX SET SPEED
        // Converts from RPMs to encoder counts per 100ms for set() method
        System.out.println("Bottom Speed Expected: " + (velocity * TALON_CPR * BOTTOM_GEAR_RATIO) / 600.0);
        System.out.println("Bottom Speed Actual: " + bottomFlywheelMaster.getSelectedSensorVelocity());
        System.out.println("Bottom Speed Error: " + (((velocity * TALON_CPR * BOTTOM_GEAR_RATIO) / 600.0) 
            - bottomFlywheelMaster.getSelectedSensorVelocity()));
        //bottomFlywheelMaster.set(ControlMode.Velocity, (velocity * TALON_CPR) / (600.0 * BOTTOM_GEAR_RATIO));
        bottomFlywheelMaster.set(ControlMode.PercentOutput, velocity);
    }

    /**
     * <h3>setTopSpeed</h3>
     * Sets the top flywheel motor speed.
     *
     * @param velocity speed of the top wheel in RPMs
     */
    public void setTopSpeed(double velocity) {
        // Converts from RPMs to encoder counts per 100ms for set() method
        //topFlywheel.set(ControlMode.Velocity, (velocity * TALON_CPR * TOP_GEAR_RATIO) / 600.0);
        topFlywheel.set(ControlMode.PercentOutput, velocity);
    }

    /**
     * <h3>getBottomSpeed</h3>
     * Gets the bottom flywheel motor speed.
     *
     * @return speed of the bottom wheel in RPMs
     */
    public double getBottomSpeed() {
        // Converts from encoder counts per 100ms to RPMs
        return (bottomFlywheelMaster.getSelectedSensorVelocity() / TALON_CPR) / BOTTOM_GEAR_RATIO * 600;
    }

    /**
     * <h3>getTopSpeed</h3>
     * Gets the top flywheel motor speed.
     *
     * @return speed of the top wheel in RPMs
     */
    public double getTopSpeed() {
        // Converts from encoder counts per 100ms to RPMs
        return (topFlywheel.getSelectedSensorVelocity() / TALON_CPR) / TOP_GEAR_RATIO * 600;
    }
}
// end of class FlywheelSubsystem