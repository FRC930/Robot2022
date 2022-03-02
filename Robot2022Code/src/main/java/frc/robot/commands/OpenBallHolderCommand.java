//-------- IMPORTS --------\\

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

//-------- COMMAND CLASS --------\\

/**
 * <h3>OpenBallHolderCommand</h3>
 * 
 * Opens the ball holder
 */
public class OpenBallHolderCommand extends CommandBase {

    //-------- VARIABLES --------\\
    private CatapultSubsystem catapultSubsystem;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>OpenBallHolderCommand</h3>
     * 
     * Initializes a new catapult command with the passed catapult subsystem
     *
     * @param catapult the {@link frc.robot.subsystems.CaptapultSubsystem
     *                 CatapultSubsystem} to use
     */
    public OpenBallHolderCommand(CatapultSubsystem catapult) {
        catapultSubsystem = catapult;
        addRequirements(catapultSubsystem);
    }

    //-------- METHODS --------\\

    @Override
    public void initialize() {
        catapultSubsystem.openBallHolder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
} // End of class BallHolderCommand
