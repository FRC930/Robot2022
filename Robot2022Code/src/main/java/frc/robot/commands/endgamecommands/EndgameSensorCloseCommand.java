package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.EndgameSensorSubsystem;

/**
 * <h3>EndgameSensorCloseCommand</h3>
 * 
 * Closes a claw when the sensor is clear
 */
public class EndgameSensorCloseCommand extends CommandBase {
    
    private final EndgamePistonSubsystem piston;
    private final EndgameSensorSubsystem sensor;

    /**
     * EndgameSensorCloseCommand
     * @param pistonSubsystem
     * 
     * @param sensorSubsystem sensor to detect
     */
    public EndgameSensorCloseCommand(EndgamePistonSubsystem pistonSubsystem, EndgameSensorSubsystem sensorSubsystem) {
        piston = pistonSubsystem;
        sensor = sensorSubsystem;
    }
    @Override
    public boolean isFinished() { // returns true when the sensor is clear
        return !sensor.isTouching();
    }

    @Override
    public void end(boolean interuppted) { // closes the claw
        if(!interuppted){
            piston.closed();
        }
        else{
            piston.open();
        }
    }

}