//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgameMotorSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameRotateUntilTouching</h3>
 * 
 * Upon being run, rotates the arm until sensor is touching.
 */
public class EndgameRotateUntilTouching extends CommandBase {

    //-------- CONSTANTS --------\\
    
    // TODO: Establish speed for endgame arm
    private final double ARM_SPEED = 0.2;

    //-------- DECLARATIONS --------\\
     
    EndgameMotorSubsystem m_MotorSubsystem;

    //-------- CONSTRUCTOR --------\\
    /**
     * EndgameRotateUntilTouching
     * 
     * @param motorSubsystem The motor to be set
     * @param sensorSubsystem The sensor whose value is used
     */ 
    public EndgameRotateUntilTouching(EndgameMotorSubsystem motorSubsystem, String sensor) {
        m_MotorSubsystem = motorSubsystem;
        addRequirements(motorSubsystem);
    }

    //-----------------------------\\
    @Override
    public void initialize() { // Starts motor when command is initialized
        m_MotorSubsystem.setMotorSpeed(ARM_SPEED);
    }

    @Override
    public void end(boolean interupted) { // Stops motor when command ends
        m_MotorSubsystem.setMotorSpeed(0.0);
    }
    /*
    @Override
    public boolean isFinished() { // Returns true when sensor's value is true
        return m_SensorSubsystem.isTouching();
    }*/
} // End of class EndgameRotateUntilTouching
