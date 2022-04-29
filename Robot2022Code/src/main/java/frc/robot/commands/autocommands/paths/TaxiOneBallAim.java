//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.AutoBase;
import frc.robot.commands.autocommands.SequentialCommands.AutoShootCargo;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;

//----- CLASS -----\\
/**
 * <h3>TaxiOneBall</h3>
 * 
 * Exits the tarmac and shoots.
 */
public class TaxiOneBallAim extends AutoBase {

    // ----- CONSTANTS -----\\

    // Movement Control
    private final static double MAX_SPEED = 0.5; // DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH
    private final static double MAX_ACCELERATION = 2.5;

    // Distance To The Center of The Hub
    private double SHOT_DISTANCE_1 = 11.68;// Figure out distance;
    // ----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    /**
     * <h3>TaxiOneBall</h3>
     * 
     * Exits the tarmac and shoots.
     * 
     * @param driveSubsystem
     * @param indexerMotorSubsystem
     * @param shooterSubsystem
     * @param intakeMotorSubsystem
     * @param intakePistonSubsystem
     * @param catapultSubsystem
     */
    public TaxiOneBallAim(DriveSubsystem driveSubsystem, IntakePistonSubsystem intakePistonSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem, ShooterSubsystem shooterSubsystem,
            ShooterHoodSubsystem shooterHoodSubsystem, IndexerMotorSubsystem indexerMotorSubsystem) {
        super(driveSubsystem, PathPlanner.loadPath("TaxiOneBall", MAX_SPEED, MAX_ACCELERATION, true));

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        // ----- TRAJECTORIES -----\\
        // Reads path file and puts it into a command for the robot to run

        this.addTrajectory(super.m_initialTrajectory);

        // ----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        Ramsete930Command r_exitTarmac = new Ramsete930Command(
                super.m_initialTrajectory,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        // ----- AUTO SEQUENCE -----\\

        addCommands(
                new WaitCommand(5.0),
                r_exitTarmac,
                new PhotonAimCommand(driveSubsystem),
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, SHOT_DISTANCE_1,
                        intakeMotorSubsystem, intakePistonSubsystem));

    } // End of Constructor
} // End of Class