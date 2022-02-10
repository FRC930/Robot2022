package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.utilities.EndgameSensorUtility;

/**
 * <h3>EndgameCloseWhenTouching</h3>
 * 
 * Closes a claw when the sensor is active
 */
public class EndgameCloseWhenTouching extends CommandBase {
    
    private final EndgamePistonSubsystem endgamePiston;
    private final int sensor;

    /**
     * EndgameCloseWhenTouching
     * @param _endgamePiston piston of claw to be closed
     * 
     * @param _endgameSensor sensor used to detect
     */
    public EndgameCloseWhenTouching(EndgamePistonSubsystem _endgamePiston, int sensorSet){
        sensor = sensorSet;
        endgamePiston = _endgamePiston;
        addRequirements(endgamePiston);
    }

    @Override
    public boolean isFinished() { // returns true when the sensor is active
        if(sensor == 2){
            return EndgameSensorUtility.getInstance().left2IsTouching() && 
            EndgameSensorUtility.getInstance().right2IsTouching();
        }
        else if(sensor == 4){
            return EndgameSensorUtility.getInstance().left4IsTouching() && 
            EndgameSensorUtility.getInstance().right4IsTouching();
        }
        else{
            return true;
        }
    }

    @Override
    public void end(boolean interuppted) { // closes the claw
        endgamePiston.closed();
    }
}
