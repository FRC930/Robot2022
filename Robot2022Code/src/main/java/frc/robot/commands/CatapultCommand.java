package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

/**
 * <h3>CatapultCommand</h3>
 * 
 * Launches the ball contained in the catapult
 * 
 * @author <a href="https://github.com/Jelombo">Jelombo</a>,
 *         <a href="https://github.com/awtpi314">awtpi314</a>
 * @since 01/20/2022
 * @version 1.0.0
 */
public class CatapultCommand extends CommandBase {
    CatapultSubsystem catapultSubsystem;

    /**
     * <h3>CatapultCommand</h3>
     * 
     * Initializes a new catapult command with the passed catapult subsystem
     *
     * @param catapult the {@link frc.robot.subsystems.CaptapultSubsystem
     *                 CatapultSubsystem} to use
     */
    public CatapultCommand(CatapultSubsystem catapult) {
        catapultSubsystem = catapult;

        addRequirements(catapultSubsystem);
    }

    @Override
    public void initialize() {
        catapultSubsystem.extend();
    }

    @Override
    public void end(boolean interrupted) {
        catapultSubsystem.retract();
    }
}
