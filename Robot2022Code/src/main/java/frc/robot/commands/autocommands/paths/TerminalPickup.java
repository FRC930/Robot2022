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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.CurrentToHubDistanceUtility;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;

//----- CLASS -----\\
/**
 * <h3>TerminalPickup</h3>
 * 
 * Four ball auto. Starts near the center of the tarmac, intakes, shoots, moves
 * to terminal, intakes, moves back to tarmac, and shoots.
 */
public class TerminalPickup extends AutoBase {

    // ----- CONSTANTS -----\\

    CurrentToHubDistanceUtility currentToHubDistanceUtility;

    // Movement Control
    // MAKE SURE THERE IS LOTS OF SPACE BEHIND TERMINAL WHEN RUNNING IN FULL SPEED
    private final static double MAX_SPEED = 5; // Set to 3 when testing
    private final static double MAX_ACCELERATION = 4; // Set to 2 when testing

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    private double SHOT_DISTANCE_1 = 10.61; // Figure out distance
    private double SHOT_DISTANCE_2 = 10.68;

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
    public TerminalPickup(
            DriveSubsystem driveSubsystem,
            IntakePistonSubsystem intakePistonSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem,
            ShooterSubsystem shooterSubsystem,
            ShooterHoodSubsystem shooterHoodSubsystem,
            IndexerMotorSubsystem indexerMotorSubsystem) {

        super(driveSubsystem, PathPlanner.loadPath("TerminalPickup1", MAX_SPEED, MAX_ACCELERATION));

        currentToHubDistanceUtility = new CurrentToHubDistanceUtility();

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        // ----- TRAJECTORIES -----\\
        this.addTrajectory(super.m_initialTrajectory);

        // Moves from tarmac to terminal to intake.
        Trajectory t_terminal = PathPlanner.loadPath("TerminalPickup2", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_terminal);

        // Moves from terminal back to tarmac to shoot.
        Trajectory t_tarmac = PathPlanner.loadPath("TerminalPickup3", MAX_SPEED, MAX_ACCELERATION, true);

        this.addTrajectory(t_tarmac);

        // ----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_taxi = new Ramsete930Command(
                super.m_initialTrajectory,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        // Creates RAMSETE Command for second trajectory
        Ramsete930Command r_terminal = new Ramsete930Command(
                t_terminal,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                driveSubsystem.getKinematics(),
                driveSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                        rightVoltage),
                driveSubsystem);

        // Creates RAMSETE Command for third trajectory
        Ramsete930Command r_tarmac = new Ramsete930Command(
                t_tarmac,
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
                new CombinedIntake(
                        intakePistonSubsystem,
                        intakeMotorSubsystem,
                        indexerMotorSubsystem,
                        r_taxi),
                new StopDrive(driveSubsystem),
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem,
                        SHOT_DISTANCE_1, intakeMotorSubsystem, intakePistonSubsystem),
                new ParallelRaceGroup(
                        new CombinedIntake(
                                intakePistonSubsystem,
                                intakeMotorSubsystem,
                                indexerMotorSubsystem),
                        new SequentialCommandGroup(
                                r_terminal,
                                new StopDrive(driveSubsystem),
                                new WaitCommand(2),
                                r_tarmac)),
                new StopDrive(driveSubsystem),
                new ParallelRaceGroup(
                        new WaitCommand(0.5)),
                new AutoShootCargo(shooterHoodSubsystem, shooterSubsystem, indexerMotorSubsystem,
                        SHOT_DISTANCE_2, intakeMotorSubsystem, intakePistonSubsystem));
    } // End of Constructor
} // End of Class