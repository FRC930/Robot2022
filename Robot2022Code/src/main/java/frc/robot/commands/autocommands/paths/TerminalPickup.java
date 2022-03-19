//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.autovisioncommands.HubAimCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

//----- CLASS -----\\
/**
 * <h3>TerminalPickup</h3>
 * 
 * Four ball auto. Starts near the center of the tarmac, intakes, shoots, moves to terminal, intakes, moves back to tarmac, and shoots.
 */
public class TerminalPickup extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    //MAKE SURE THERE IS LOTS OF SPACE BEHIND TERMINAL WHEN RUNNING IN FULL SPEED
    private final double MAX_SPEED = 5; //Set to 3 when testing
    private final double MAX_ACCELERATION = 4; //Set to 2 when testing

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>TerminalPickup</h3>
     * 
     * Four ball auto. Starts near the center of the tarmac, intakes, shoots, moves to terminal, intakes, moves back to tarmac, and shoots.
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
        FlywheelSubsystem flywheelSubsystem,
        IndexerMotorSubsystem indexerMotorSubsystem
    ) {

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\

        // Exits the tarmac for a taxi, intakes, and shoots.
        Trajectory t_taxi = PathPlanner.loadPath("TerminalPickup1", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_taxi);

        // Moves from tarmac to terminal to intake.
        Trajectory t_terminal = PathPlanner.loadPath("TerminalPickup2", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_terminal);

        // Moves from terminal back to tarmac to shoot.
        Trajectory t_tarmac = PathPlanner.loadPath("TerminalPickup3", MAX_SPEED, MAX_ACCELERATION, true);

        this.addTrajectory(t_tarmac);

        //----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_taxi = new Ramsete930Command(
            t_taxi,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                    rightVoltage),
            driveSubsystem
        );

        // Creates RAMSETE Command for second trajectory
        Ramsete930Command r_terminal = new Ramsete930Command(
            t_terminal,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                    rightVoltage),
            driveSubsystem
        );

        // Creates RAMSETE Command for third trajectory
        Ramsete930Command r_tarmac = new Ramsete930Command(
            t_tarmac,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                    rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\

        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends
        // Sets catapult shot to short
        // reset the encoders and odometry to where our path starts
        // starts the path
        // engages the intake piston runs them at the same time
        // it stops driving.
        addCommands(
            new ResetAutonomousCommand(t_taxi.getInitialPose(), driveSubsystem),
            new ParallelRaceGroup(
                new EngageIntakePistonsCommand(intakePistonSubsystem),
                new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
                r_taxi
            ),
            new StopDrive(driveSubsystem),
            new ParallelRaceGroup(
                new HubAimCommand(driveSubsystem),
                new WaitCommand(0.5)
            ),
            new ParallelRaceGroup(new ShootCargoCommand(flywheelSubsystem, indexerMotorSubsystem), new WaitCommand(1)),
            new ParallelRaceGroup(new ShootCargoCommand(flywheelSubsystem, indexerMotorSubsystem), new WaitCommand(1)),

            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new EngageIntakePistonsCommand(intakePistonSubsystem),
                    new RunIntakeMotorsCommand(intakeMotorSubsystem, false)
                ),
                new SequentialCommandGroup(
                    r_terminal,
                    new StopDrive(driveSubsystem),
                    new WaitCommand(2),
                    r_tarmac
                )
            ),
            new StopDrive(driveSubsystem),
            new ParallelRaceGroup(
                new HubAimCommand(driveSubsystem),
                new WaitCommand(0.5)
            ),
            new ParallelRaceGroup(new ShootCargoCommand(flywheelSubsystem, indexerMotorSubsystem), new WaitCommand(1)),
            new ParallelRaceGroup(new ShootCargoCommand(flywheelSubsystem, indexerMotorSubsystem), new WaitCommand(1))
            );
    } // End of Constructor
} // End of Class