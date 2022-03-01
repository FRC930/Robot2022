/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\
package frc.robot.commands;

import edu.wpi.first.wpilibj.MotorSafety;
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

    private final double MOTOR_SPEED = 0.4;
    // Delay between the closing of a sensor circuit and
    // re-activating the motor after launching ball
    private final double RESTART_DELAY = 25; 

    // -------- VARIABLES --------\\

    private final IndexerMotorSubsystem motor;
    private int counter;
    private final BallSensorUtility sensorUtility = BallSensorUtility.getInstance();
    private boolean reversed = false;
    private double lastSpeed = -2.0; //set to speed that would have never be set to before

    // -------- CONSTRUCTOR --------\\
    /**
     * <h3>IndexerForwardCommand</h3>
     *
     * Sets the motor speed of the indexer
     *
     * @param _motor The motor on the indexer
     */
    public IndexerForwardCommand(IndexerMotorSubsystem _motor, boolean isReversed) {
        motor = _motor;
        reversed = isReversed;
        counter = 0;
        addRequirements(motor);
    }

    // -------- COMMANDBASE METHODS --------\\

    public void initialize(){
        lastSpeed=-2.0; //set to speed that would have never be set to before
        if(reversed){
            setSpeed(-MOTOR_SPEED);
        }
    }
    /**
     * <h3>execute</h3>
     *
     * If the catapult sensor does not detect a ball it sets the motor speed to
     * MOTOR_SPEED.
     * If it does detect a ball it sets the motor speed to 0
     */
    public void execute() {
        if (!reversed) {
            if ((!sensorUtility.catapultIsTripped()
                    || !sensorUtility.indexerIsTripped())) {
                // If past delay loop o
                if(counter > RESTART_DELAY) {
                    setSpeed(MOTOR_SPEED);
                    counter = 0;
                } else {
                    counter++;
                }
            } else {
                counter = 0;
                setSpeed(0.0);
            }
        }
    }

    private void setSpeed(double motorSpeed) {
        //System.out.println("lastSpeed"+lastSpeed);
        if(lastSpeed != motorSpeed) {
            motor.setMotorSpeed(motorSpeed);
            System.out.println("Speed"+motorSpeed);
        }
        lastSpeed = motorSpeed;
    }
    
    /**
     * <h3>end</h3>
     *
     * called when the method ends
     */
    @Override
    public void end(boolean interrupted) { // sets the motor speed to 0
        setSpeed(0.0);
    }

    @Override
    public boolean isFinished() { // when true, ends command
        return false;
    }

} // End of IndexerForwardCommand