package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.utilities.BallSensorUtility;

public class IndexerForwardCommand extends CommandBase{
    
    private final double MOTOR_SPEED = 0.5;
    private final IndexerMotorSubsystem motor;

    public IndexerForwardCommand(IndexerMotorSubsystem _motor){
        motor = _motor;
        addRequirements(motor);
    }

    /**
     * <h3>execute</h3>
     *
     * If the catapult sensor does not detect a ball it sets the motor speed to MOTOR_SPEED.
     * If it does detect a ball it sets the motor speed to 0
     */
    public void execute() {
        if (!BallSensorUtility.getInstance().catapultIsTripped()) {
            motor.setMotorSpeed(MOTOR_SPEED);
        } else { 
            motor.setMotorSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted){
        motor.setMotorSpeed(0.0);  
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
