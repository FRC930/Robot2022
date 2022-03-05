//-------- IMPORTS --------\\

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.utilities.BallSensorUtility;

//-------- COMMAND CLASS --------\\

/**
 * <h3>BallHolderCommand</h3>
 * 
 * Manages the ball holder based off of catapult sensor
 */
public class BallHolderCommand extends CommandBase {

    // -------- CONSTANTS --------\\

    private final int CLOSE_DELAY = 3;
    //private final int OPEN_DELAY = 3;

    // -------- VARIABLES --------\\

    private CatapultSubsystem catapultSubsystem;
    private int counter;
    //private int counter2;
    private boolean isAuton;
    private final BallSensorUtility sensorUtility = BallSensorUtility.getInstance();

    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>BallHolderCommand</h3>
     * 
     * Initializes a new catapult command with the passed catapult subsystem
     *
     * @param catapult the {@link frc.robot.subsystems.CaptapultSubsystem
     *                 CatapultSubsystem} to use
     * @param isAuton  argument should be true if command is being run during
     *                 autonomous, else false
     */
    public BallHolderCommand(CatapultSubsystem catapult, boolean isAuton) {
        catapultSubsystem = catapult;
        this.isAuton = isAuton;
        counter = 0;
        addRequirements(catapultSubsystem);
    }

    /**
     * <h3>BallHolderCommand</h3>
     * 
     * Initializes a new catapult command with the passed catapult subsystem
     * USE ONLY FOR TELEOP
     *
     * @param catapult the {@link frc.robot.subsystems.CaptapultSubsystem
     *                 CatapultSubsystem} to use
     */
    public BallHolderCommand(CatapultSubsystem catapult) {
        this(catapult, false);
    }

    // -------- METHODS --------\\

    @Override
    public void initialize() {
        catapultSubsystem.openBallHolder();
    }

    @Override
    public void execute() {
        if (sensorUtility.catapultIsTripped() && counter > CLOSE_DELAY) {
            catapultSubsystem.closeBallHolder();
            counter = 0;
        } else if (sensorUtility.catapultIsTripped()) {
            counter++;
        } /*else if (!sensorUtility.catapultIsTripped() && counter2 > OPEN_DELAY) {
            catapultSubsystem.openBallHolder();
            counter2 = 0;
        } else if(!sensorUtility.catapultIsTripped()) {
            counter = 0;
            counter2++;
        }*/
        else {
            counter = 0;
            catapultSubsystem.openBallHolder();
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (!isAuton) {
            catapultSubsystem.openBallHolder();
            catapultSubsystem.retractRetractor();
        }
    }

    @Override
    public boolean isFinished() {
        if (isAuton) {
            return catapultSubsystem.ballHolderIsClosed();
        } else {
            return false;
        }
    }
} // End of class BallHolderCommand
