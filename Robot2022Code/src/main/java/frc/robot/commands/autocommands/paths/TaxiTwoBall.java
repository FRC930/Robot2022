//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.AutoBase;
import frc.robot.commands.autocommands.SequentialCommands.AutoShootCargo;
import frc.robot.commands.autocommands.SequentialCommands.CombinedIntake;
import frc.robot.commands.autocommands.SequentialCommands.StopDrive;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;

//----- CLASS -----\\
/**
 * <h3>TaxiTwoBall</h3>
 * 
 * Exits the tarmac, intakes, and shoots.
 */
public class TaxiTwoBall extends AutoBase {

    //----- CONSTANTS -----\\

    // Movement Control
    private final static double MAX_SPEED = 1.0;
    private final static double MAX_ACCELERATION = 1.0;

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>TaxiTwoBall</h3>
     * 
     * Exits the tarmac, intakes, and shoots.
     * 
     * @param driveSubsystem
     * @param intakePistonSubsystem
     * @param intakeMotorSubsystem
     * @param visionCameraSubsystem
     * @param catapultSubsystem
     */
    public TaxiTwoBall(
        DriveSubsystem driveSubsystem,
        IntakePistonSubsystem intakePistonSubsystem,
        IntakeMotorSubsystem intakeMotorSubsystem,
        ShooterSubsystem shooterSubsystem,
        ShooterHoodSubsystem shooterHoodSubsystem,
        IndexerMotorSubsystem indexerMotorSubsystem
    ) {

        super(driveSubsystem, PathPlanner.loadPath("TaxiTwoBall", MAX_SPEED, MAX_ACCELERATION));

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\
        // Robot exits the tarmac, intakes, and shoots
        this.addTrajectory(super.m_initialTrajectory);

        SmartDashboard.putString("Pos1", super.m_initialTrajectory.getInitialPose().toString());
        SmartDashboard.putString("current Gyro Position", m_odometry.getPoseMeters().toString());

        //----- RAMSETE COMMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        Ramsete930Command r_exitTarmac = new Ramsete930Command(
            super.m_initialTrajectory,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\
        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command endst

        addCommands(
            new CombinedIntake(
                intakePistonSubsystem,
                intakeMotorSubsystem,
                indexerMotorSubsystem,
                r_exitTarmac
            ),
            new StopDrive(driveSubsystem),
            new PhotonAimCommand(driveSubsystem).withTimeout(2),
            new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, 11.62 , intakeMotorSubsystem, intakePistonSubsystem, ShootCargoCommand.SHOOT_TIME));
        

    } // End of Constructor
} // End of Class