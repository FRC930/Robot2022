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
     * 
     * @param shooterHood The subsystem of the shooter's hood
     */
    public UpdateHoodCommand(ShooterHoodSubsystem shooterHood) {
        shooterHoodSubsystem = shooterHood;
    }

    @Override
    public void initialize() {
        if (shooterHoodSubsystem.getHoodPosition() < 0.04) {
            shooterHoodSubsystem.setHoodPosition(0.08333333333333333);
        } else if (shooterHoodSubsystem.getHoodPosition() >= 0.04) {
            shooterHoodSubsystem.setHoodPosition(0.0);
        }
    }

    @Override
    public void execute() {
        System.out.println("Hood Pos: " + shooterHoodSubsystem.getHoodPosition());
    }
}
