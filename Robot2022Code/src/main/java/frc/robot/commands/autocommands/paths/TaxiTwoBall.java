//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.autocommands.SequentialCommands.AutoShootCargo;
import frc.robot.commands.autocommands.SequentialCommands.CombinedIntake;
import frc.robot.commands.autocommands.SequentialCommands.StopDrive;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.CurrentToHubDistanceUtility;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
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
public class TaxiTwoBall extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    private final double MAX_SPEED = 1.0;
    private final double MAX_ACCELERATION = 1.0;

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;
    private final CurrentToHubDistanceUtility currentToHubDistanceUtility;

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
        currentToHubDistanceUtility = new CurrentToHubDistanceUtility();

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\

        // Robot exits the tarmac, intakes, and shoots
        Trajectory t_exitTarmac = PathPlanner.loadPath("TaxiTwoBall", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_exitTarmac);

        SmartDashboard.putString("Pos1", t_exitTarmac.getInitialPose().toString());
        SmartDashboard.putString("current Gyro Position", m_odometry.getPoseMeters().toString());

        //----- RAMSETE COMMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        Ramsete930Command r_exitTarmac = new Ramsete930Command(
            t_exitTarmac,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\
        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends

        addCommands(
            new ResetAutonomousCommand(t_exitTarmac.getInitialPose(), driveSubsystem),
            new CombinedIntake(
                intakePistonSubsystem,
                intakeMotorSubsystem,
                indexerMotorSubsystem,
                r_exitTarmac
            ),
            new StopDrive(driveSubsystem),
            new PhotonAimCommand(driveSubsystem),
            new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, currentToHubDistanceUtility.getDistanceToHub(driveSubsystem.getOdometry().getPoseMeters()), intakeMotorSubsystem, intakePistonSubsystem, ShootCargoCommand.SHOOT_TIME));
        

    } // End of Constructor
} // End of Class