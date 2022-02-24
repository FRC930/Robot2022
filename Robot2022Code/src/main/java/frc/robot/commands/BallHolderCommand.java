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

    //-------- CONSTANTS --------\\

    private final int CLOSE_DELAY = 25;

    //-------- VARIABLES --------\\

    private CatapultSubsystem catapultSubsystem;
    private int counter;

    //-------- CONSTRUCTOR --------\\

    /**
     * <h3>BallHolderCommand</h3>
     * 
     * Initializes a new catapult command with the passed catapult subsystem
     *
     * @param catapult the {@link frc.robot.subsystems.CaptapultSubsystem
     *                 CatapultSubsystem} to use
     */
    public BallHolderCommand(CatapultSubsystem catapult) {
        catapultSubsystem = catapult;
        counter = 0;
        addRequirements(catapultSubsystem);
    }

    //-------- METHODS --------\\

    @Override
    public void initialize() {
        catapultSubsystem.openBallHolder();
    }

    @Override
    public void execute() {
        if (BallSensorUtility.getInstance().catapultIsTripped() && counter > CLOSE_DELAY) {
            catapultSubsystem.closeBallHolder();
            counter = 0;
        } else if (BallSensorUtility.getInstance().catapultIsTripped()) {
            counter++;
        } else {
            counter = 0;
            catapultSubsystem.openBallHolder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        catapultSubsystem.openBallHolder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
} // End of class BallHolderCommand
