package frc.robot.commands.autocommands.SequentialCommands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.shootercommands.AdjustHoodCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.GyroUtility;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.utilities.ShooterUtility;

public class AutoShootCargo extends PathPlannerSequentialCommandGroupUtility {
    Pigeon2 pigeon2 = GyroUtility.getInstance().getGyro();
    public AutoShootCargo(ShooterHoodSubsystem shooterHoodSubsystem, ShooterSubsystem shooterSubsystem, IndexerMotorSubsystem indexerMotorSubsystem, Double DISTANCE){
        
        
        addCommands(
        new AdjustHoodCommand(shooterHoodSubsystem, ShooterUtility.calculateHoodPos(DISTANCE)),
        new ShootCargoCommand(shooterSubsystem, indexerMotorSubsystem,
                ShooterUtility.calculateTopSpeed(DISTANCE),
                ShooterUtility.calculateBottomSpeed(DISTANCE))
                        .withTimeout(ShootCargoCommand.SHOOT_TIME)
                        );
            
        
    
    }
    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("gyro value", pigeon2.getYaw());
    }
}
