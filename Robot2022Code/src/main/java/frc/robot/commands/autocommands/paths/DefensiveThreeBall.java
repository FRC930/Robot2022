//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.AutoBase;
import frc.robot.commands.autocommands.SequentialCommands.AutoShootCargo;
import frc.robot.commands.autocommands.SequentialCommands.CombinedIntake;
import frc.robot.commands.autocommands.SequentialCommands.StopDrive;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;

//----- CLASS -----\\
/**
 * <h3>DefensiveTwoBall</h3>
 * 
 * Exits the tarmac, intakes, and shoots. Moves to adjacent enemy cargo, intakes
 * it, and shoots it into the hangar zone.
 */
public class DefensiveThreeBall extends AutoBase {

    // ----- CONSTANTS -----\\

    // Movement Control
    private final static double MAX_SPEED = 1.0;
    private final static double MAX_ACCELERATION = 1.0;

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    // Distance To The Center of The Hub
    private double SHOT_DISTANCE_1 = 11.68;
    private double SHOT_DISTANCE_2 = 8.0;

    // ----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;
    // private final CurrentToHubDistanceUtility currentToHubDistanceUtility;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>DefensiveTwoBall</h3>
     * 
     * Exits the tarmac, intakes, and shoots. Moves to adjacent enemy cargo, intakes
     * it, and shoots it into the hangar zone.
     * 
     * @param driveSubsystem
     * @param intakePistonSubsystem
     * @param intakeMotorSubsystem
     * @param visionCameraSubsystem
     * @param catapultSubsystem
     */
    public DefensiveThreeBall(
            DriveSubsystem driveSubsystem,
            IntakePistonSubsystem intakePistonSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem,
            ShooterSubsystem shooterSubsystem,
            ShooterHoodSubsystem shooterHoodSubsystem,
            IndexerMotorSubsystem indexerMotorSubsystem) {
        super(driveSubsystem, PathPlanner.loadPath("DefensiveThreeBall1", MAX_SPEED, MAX_ACCELERATION));

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();
        // ----- TRAJECTORIES -----\\
        // Reads path file and puts it into a command for the robot to run

        // Robot exits the tarmac, intakes, and shoots

        // Robot approaches the adjacent enemy cargo and shoots it into the hangar zone.
        Trajectory t_adjacentEnemyCargo = PathPlanner.loadPath("DefensiveThreeBall2", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(super.m_initialTrajectory);
        this.addTrajectory(t_adjacentEnemyCargo);

        SmartDashboard.putString("Pos1", super.m_initialTrajectory.getInitialPose().toString());
        SmartDashboard.putString("current Gyro Position", m_odometry.getPoseMeters().toString());

        // ----- RAMSETE COMMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_exitTarmac = new Ramsete930Command(
                super.m_initialTrajectory,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
                driveSubsystem);

        Ramsete930Command r_adjacentEnemyCargo = new Ramsete930Command(
                t_adjacentEnemyCargo,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
                driveSubsystem);

        // ----- AUTO SEQUENCE -----\\
        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends
        // Line up left side of the robot with middle of the tarmac, front right bumper
        // is on the end of the tarmac

        addCommands(
                new CombinedIntake(
                        intakePistonSubsystem,
                        intakeMotorSubsystem,
                        indexerMotorSubsystem,
                        r_exitTarmac),
                new StopDrive(driveSubsystem),
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, SHOT_DISTANCE_1,
                        intakeMotorSubsystem, intakePistonSubsystem),
                new CombinedIntake(
                        intakePistonSubsystem,
                        intakeMotorSubsystem,
                        indexerMotorSubsystem,
                        r_adjacentEnemyCargo),
                new StopDrive(driveSubsystem), // hangar point for manual calculation should be around x:1.65 y:6.55
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, SHOT_DISTANCE_2,
                        intakeMotorSubsystem, intakePistonSubsystem));

    } // End of Constructor
} // End of Class