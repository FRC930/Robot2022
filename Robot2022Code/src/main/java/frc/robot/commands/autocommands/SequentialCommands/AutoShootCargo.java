package frc.robot.commands.autocommands.SequentialCommands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

/**
 * <h3>AutoShootCargo</h3>
 * This command creates a full shooting sequence to be used in autonomous.
 */
public class AutoShootCargo extends PathPlannerSequentialCommandGroupUtility {

    private static final double AUTON_SHOOT_TIME = 1.0;

    Pigeon2 pigeon2 = GyroUtility.getInstance().getGyro();

    /**
     * Creates an auton shoot command with default shoot time.
     * 
     * @param shooterHoodSubsystem
     * @param shooterSubsystem
     * @param indexerMotorSubsystem
     * @param distance
     * @param intakeMotorSubsystem
     * @param intakePistonSubsystem
     */
    public AutoShootCargo(ShooterHoodSubsystem shooterHoodSubsystem, ShooterSubsystem shooterSubsystem,
            IndexerMotorSubsystem indexerMotorSubsystem, Double distance, IntakeMotorSubsystem intakeMotorSubsystem,
            IntakePistonSubsystem intakePistonSubsystem) {
        this(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, distance, intakeMotorSubsystem,
                intakePistonSubsystem, AUTON_SHOOT_TIME);
    }

    /**
     * Creates an auton shoot command with custom shoot time.
     * 
     * @param shooterHoodSubsystem
     * @param shooterSubsystem
     * @param indexerMotorSubsystem
     * @param distance
     * @param intakeMotorSubsystem
     * @param intakePistonSubsystem
     * @param shootTime
     */
    public AutoShootCargo(ShooterHoodSubsystem shooterHoodSubsystem, ShooterSubsystem shooterSubsystem,
            IndexerMotorSubsystem indexerMotorSubsystem, Double distance, IntakeMotorSubsystem intakeMotorSubsystem,
            IntakePistonSubsystem intakePistonSubsystem, Double shootTime) {
        addCommands(
                new AdjustHoodCommand(shooterHoodSubsystem, ShooterUtility.calculateHoodPos(distance)),
                new ParallelRaceGroup(
                        new ShootCargoCommand(shooterSubsystem, indexerMotorSubsystem,
                                ShooterUtility.calculateTopSpeed(distance),
                                ShooterUtility.calculateBottomSpeed(distance)).withTimeout(shootTime)));
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("gyro value", pigeon2.getYaw());
    }
}
