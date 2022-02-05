package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerMotorSubsystem;

public class IndexerForwardCommand extends CommandBase{
    
    private final double MOTOR_SPEED = 0.2;
    private final IndexerMotorSubsystem motor;

    public IndexerForwardCommand(IndexerMotorSubsystem _motor){
        motor = _motor;
        addRequirements(motor);
    }

    @Override
    public void initialize(){
        motor.setMotorSpeed(MOTOR_SPEED);
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
