package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.utilities.BallSensorUtility;

/**
 * <h3>BallHolderCommand</h3>
 * 
 * Manages the ball holder based off of catapult sensor
 */
public class BallHolderCommand extends CommandBase {
    CatapultSubsystem catapultSubsystem;

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

        addRequirements(catapultSubsystem);
    }

    @Override
    public void initialize() {
        catapultSubsystem.openBallHolder();
    }

    @Override
    public void execute() {
        if (BallSensorUtility.getInstance().catapultIsTripped()) {
            catapultSubsystem.closeBallHolder();
        } else {
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
}
