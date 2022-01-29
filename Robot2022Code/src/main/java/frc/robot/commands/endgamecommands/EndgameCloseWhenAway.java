package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.EndgameSensorSubsystem;

/**
 * <h3>EndgameCloseWhenAway</h3>
 * 
 * Closes a claw when the sensor is clear
 */
public class EndgameCloseWhenAway extends CommandBase {
    
    private final EndgameSensorSubsystem endgameSensor;
    private final EndgamePistonSubsystem endgamePiston;

    /**
     * EndgameCloseWhenAway
     * @param _endgamePiston piston of claw to be closed
     * 
     * @param _endgameSensor sensor used to detect
     */
    public EndgameCloseWhenAway(EndgamePistonSubsystem _endgamePiston, EndgameSensorSubsystem _endgameSensor) {
        endgamePiston = _endgamePiston;
        endgameSensor = _endgameSensor;
        addRequirements(endgamePiston, endgameSensor);
    }

    @Override
    public boolean isFinished() { // returns true when the sensor is clear
        return !endgameSensor.isTouching();
    }

    @Override
    public void end(boolean interuppted) { // closes the claw
        endgamePiston.closed();
    }

}