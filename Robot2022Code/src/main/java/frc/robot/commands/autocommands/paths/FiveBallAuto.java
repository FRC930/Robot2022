//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.autocommands.SequentialCommands.AutoShootCargo;
import frc.robot.commands.autocommands.SequentialCommands.CombinedIntake;
import frc.robot.commands.autocommands.SequentialCommands.StopDrive;
import frc.robot.commands.autovisioncommands.HubAimCommand;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.DisengageIntakePistonsCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
 * <h3>FiveBallAuto</h3>
 * 
 * Four ball auto. Starts near the center of the tarmac, intakes, shoots, moves
 * to terminal, intakes, moves back to tarmac, and shoots.
 */
public class FiveBallAuto extends PathPlannerSequentialCommandGroupUtility {

    // ----- CONSTANTS -----\\

    CurrentToHubDistanceUtility currentToHubDistanceUtility;

    // Movement Control
    // MAKE SURE THERE IS LOTS OF SPACE BEHIND TERMINAL WHEN RUNNING IN FULL SPEED
    private final double MAX_SPEED = 4; // Set to 3 when testing
    private final double MAX_ACCELERATION = 2; // Set to 2 when testing

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    private final double SHOT_DISTANCE1 = 9.8;
    private final double SHOT_DISTANCE2 = 9.85;
    private final double SHOT_DISTANCE3 = 9.7;

    

    // ----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>TerminalPickup</h3>
     * 
     * Four ball auto. Starts near the center of the tarmac, intakes, shoots, moves
     * to terminal, intakes, moves back to tarmac, and shoots.
     * 
     * @param driveSubsystem
     * @param intakePistonSubsystem
     * @param intakeMotorSubsystem
     * @param visionCameraSubsystem
     * @param catapultSubsystem
     */
    public FiveBallAuto(
            DriveSubsystem driveSubsystem,
            IntakePistonSubsystem intakePistonSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem,
            ShooterSubsystem shooterSubsystem,
            ShooterHoodSubsystem shooterHoodSubsystem,
            IndexerMotorSubsystem indexerMotorSubsystem) {
        currentToHubDistanceUtility = new CurrentToHubDistanceUtility();

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        // ----- TRAJECTORIES -----\\

        // Exits the tarmac for a taxi, intakes, and shoots.
        Trajectory t_path1 = PathPlanner.loadPath("FiveBallAuto1", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_path1);

        // Moves from tarmac to terminal to intake.
        Trajectory t_path2 = PathPlanner.loadPath("FiveBallAuto2", MAX_SPEED, MAX_ACCELERATION, true);

        this.addTrajectory(t_path2);

        // Moves from terminal back to tarmac to shoot.
        Trajectory t_path3 = PathPlanner.loadPath("FiveBallAuto3", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_path3);

        Trajectory t_path4 = PathPlanner.loadPath("FiveBallAuto4", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_path4);

        Trajectory t_path5 = PathPlanner.loadPath("FiveBallAuto5", MAX_SPEED, MAX_ACCELERATION, true);

        this.addTrajectory(t_path5);
        // ----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_path1 = new Ramsete930Command(
                t_path1,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        // Creates RAMSETE Command for second trajectory
        Ramsete930Command r_path2 = new Ramsete930Command(
                t_path2,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        // Creates RAMSETE Command for third trajectory
        Ramsete930Command r_path3 = new Ramsete930Command(
                t_path3,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        Ramsete930Command r_path4 = new Ramsete930Command(
                t_path4,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        Ramsete930Command r_path5 = new Ramsete930Command(
                t_path5,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        // ----- AUTO SEQUENCE -----\\

        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends
        // Sets catapult shot to short
        // reset the encoders and odometry to where our path starts
        // starts the path
        // engages the intake piston runs them at the same time
        // it stops driving.
        addCommands(
                new ResetAutonomousCommand(t_path1.getInitialPose(), driveSubsystem),
                new CombinedIntake(
                        intakePistonSubsystem,
                        intakeMotorSubsystem,
                        indexerMotorSubsystem,
                        r_path1),
                new StopDrive(driveSubsystem),
                        new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, SHOT_DISTANCE1, intakeMotorSubsystem, intakePistonSubsystem, 0.75),
                new CombinedIntake(intakePistonSubsystem, intakeMotorSubsystem, indexerMotorSubsystem, r_path2),
                new CombinedIntake(intakePistonSubsystem, intakeMotorSubsystem, indexerMotorSubsystem, r_path3),
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, SHOT_DISTANCE2, intakeMotorSubsystem, intakePistonSubsystem, 0.75),
                new CombinedIntake(intakePistonSubsystem, intakeMotorSubsystem, indexerMotorSubsystem, r_path4),
                new ParallelRaceGroup(new CombinedIntake(intakePistonSubsystem, intakeMotorSubsystem, indexerMotorSubsystem),new WaitCommand(1)),
                new CombinedIntake(intakePistonSubsystem, intakeMotorSubsystem, indexerMotorSubsystem, r_path5),
                new StopDrive(driveSubsystem),
                new DisengageIntakePistonsCommand(intakePistonSubsystem).withTimeout(0.1),
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem, SHOT_DISTANCE3, intakeMotorSubsystem, intakePistonSubsystem, 0.75)
        );
    } // End of Constructor
} // End of Class