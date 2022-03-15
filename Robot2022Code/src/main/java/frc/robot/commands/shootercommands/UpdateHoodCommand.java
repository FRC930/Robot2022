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
    private final double pos;

    /**
     * <h3>UpdateHoodCommand</h3>
     * 
     * Updates position of hood according to needed angle
     * 
     * @param shooterHood The subsystem of the shooter's hood
     */
    public UpdateHoodCommand(ShooterHoodSubsystem shooterHood, double pos) {
        shooterHoodSubsystem = shooterHood;
        this.pos = pos;
    }

    @Override
    public void initialize() {
        shooterHoodSubsystem.setHoodPosition(pos);
    }

    @Override
    public void execute() {
        //System.out.println("Hood Pos: " + shooterHoodSubsystem.getHoodPosition());
        //System.out.println("Hood Error: " + (shooterHoodSubsystem.getHoodPosition() - 0.03));
    }
}
