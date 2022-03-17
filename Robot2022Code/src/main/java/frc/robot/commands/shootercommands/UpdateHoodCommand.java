package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
     * Updates position of hood
     * 
     * @param shooterHood The subsystem of the shooter's hood
     * @param pos The position to set the hood. Set -1 to use Shuffleboard.
     */
    public UpdateHoodCommand(ShooterHoodSubsystem shooterHood, double pos) {
        shooterHoodSubsystem = shooterHood;
        this.pos = pos;
    }

    @Override
    public void initialize() {
        // Sets the position for the hood
        // NOTE: without stopHood() call in end(), PID will hold set position
        if(pos != -1){
            shooterHoodSubsystem.setHoodPosition(pos);
        }
        else{
            // Sets position using shuffleboard
            shooterHoodSubsystem.setHoodPosition(SmartDashboard.getNumber("Hood Position", 0));
        }
    }

    @Override
    public void execute() {
        //System.out.println("Hood Pos: " + shooterHoodSubsystem.getHoodPosition());
        //System.out.println("Hood Error: " + (shooterHoodSubsystem.getHoodPosition() - 0.03));
    }
}
