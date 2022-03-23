package frc.robot.commands.autocommands.SequentialCommands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.shootercommands.AdjustHoodCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.GyroUtility;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.utilities.ShooterUtility;

public class AutoShootCargo extends PathPlannerSequentialCommandGroupUtility {
    Pigeon2 pigeon2 = GyroUtility.getInstance().getGyro();
    
    public AutoShootCargo(ShooterHoodSubsystem shooterHoodSubsystem, ShooterSubsystem shooterSubsystem, IndexerMotorSubsystem indexerMotorSubsystem, Double DISTANCE, IntakeMotorSubsystem intakeMotorSubsystem, IntakePistonSubsystem intakePistonSubsystem, Double SHOOT_TIME){
        
        
        addCommands(
        new AdjustHoodCommand(shooterHoodSubsystem, ShooterUtility.calculateHoodPos(DISTANCE)),
        new ParallelRaceGroup(
            new ShootCargoCommand(shooterSubsystem, indexerMotorSubsystem,
            ShooterUtility.calculateTopSpeed(DISTANCE),
            ShooterUtility.calculateBottomSpeed(DISTANCE)).withTimeout(SHOOT_TIME)
            )
        );
        
        
    
    }
    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("gyro value", pigeon2.getYaw());
    }
}
