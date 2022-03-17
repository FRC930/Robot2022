/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.utilities.BallSensorUtility;

//-------- COMMAND CLASS --------\\
/**
 * <h3>IndexerForwardCommand</h3>
 * 
 * Sets the motor speed of the indexer
 */
public class IndexerForwardCommand extends CommandBase {

    // -------- CONSTANTS --------\\
    private final double INTAKE_MOTOR_SPEED = 0.75;
    private final double LOADED_MOTOR_SPEED = 0.35;

    // -------- VARIABLES --------\\

    private final IndexerMotorSubsystem indexerMotor;
    // private int counter;
    private final BallSensorUtility sensorUtility = BallSensorUtility.getInstance();
    private boolean reversed = false;
    private double lastIntakeSpeed = -2.0; // set to speed that would have never be set to before
    private double lastLoadedSpeed = -2.0; // set to speed that would have never be set to before

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>IndexerForwardCommand</h3>
     *
     * Sets the motor speed of the indexer
     *
     * @param indexer The motor on the indexer
     */
    public IndexerForwardCommand(IndexerMotorSubsystem indexer, boolean isReversed) {
        indexerMotor = indexer;
        reversed = isReversed;
        SmartDashboard.putNumber("Intake Indexer Speed", INTAKE_MOTOR_SPEED);
        SmartDashboard.putNumber("Loaded Indexer Speed", LOADED_MOTOR_SPEED);
        addRequirements(indexer);
    }

    // -------- COMMANDBASE METHODS --------\\

    public void initialize() {
        lastIntakeSpeed = -2.0; // Set last speed outside of usable range
        lastLoadedSpeed = -2.0;
        if (reversed) {
            indexerMotor.setIntakeMotorSpeed(-LOADED_MOTOR_SPEED);
            indexerMotor.setLoadedMotorSpeed(-LOADED_MOTOR_SPEED);
        } else {
            indexerMotor.setIntakeMotorSpeed(0.0);
            indexerMotor.setLoadedMotorSpeed(0.0);
        }
    }

    /**
     * <h3>execute</h3>
     *
     * If the loaded sensor does not detect a ball it sets the motor speed to
     * MOTOR_SPEED.
     * If it does detect a ball it sets the motor speed to 0
     */
    public void execute() {
        if (!reversed) {
            if (sensorUtility.loadedIsTripped()) {
                setLoadedSpeed(0.0);
            } else {
                setLoadedSpeed(SmartDashboard.getNumber("Loaded Indexer Speed", 0));
            }
            if (!sensorUtility.intakeIsTripped() || !sensorUtility.loadedIsTripped()) {
                setIntakeSpeed(SmartDashboard.getNumber("Intake Indexer Speed", 0));
            } else {
                setIntakeSpeed(0.0);
            }
        }
    }

    private void setIntakeSpeed(double speed) {
        if (lastIntakeSpeed != speed) {
            indexerMotor.setIntakeMotorSpeed(speed);
            lastIntakeSpeed = speed;
        }
    }

    private void setLoadedSpeed(double speed) {
        if (lastLoadedSpeed != speed) {
            indexerMotor.setLoadedMotorSpeed(speed);
            lastLoadedSpeed = speed;
        }
    }

    /**
     * <h3>end</h3>
     *
     * called when the method ends
     */
    @Override
    public void end(boolean interrupted) { // sets the motor speed to 0
        indexerMotor.setIntakeMotorSpeed(0.0);
        indexerMotor.setLoadedMotorSpeed(0.0);
    }

    @Override
    public boolean isFinished() { // when true, ends command
        return false;
    }

} // End of IndexerForwardCommand