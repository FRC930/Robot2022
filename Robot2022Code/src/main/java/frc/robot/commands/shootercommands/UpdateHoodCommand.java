package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHoodSubsystem;

/**
 * <h3>UpdateHoodCommand</h3>
 * 
 * Updates position of hood according to needed angle
 */
public class UpdateHoodCommand extends CommandBase {
    
    private final ShooterHoodSubsystem shooterHoodSubsystem;

    /**
     * <h3>UpdateHoodCommand</h3>
     * 
     * Updates position of hood according to needed angle
     * @param shooterHood The subsystem of the shooter's hood
     */
    public UpdateHoodCommand(ShooterHoodSubsystem shooterHood) {
        shooterHoodSubsystem = shooterHood;
    }

    @Override
    public void execute() {
        // TODO: Add logic
    }
}
