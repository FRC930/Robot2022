//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.AutonomousAimCommand;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;

//----- CLASS -----\\
/**
 * <h3>ShootMoveShoot</h3>
 * 
 * Shoots, exits tarmac, and shoots again.
 */
public class ShootMoveShoot extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    private final double MAX_SPEED = 0.5;
    private final double MAX_ACCELERATION = 0.5;

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>ShootMoveShoot</h3>
     * 
     * Shoots, exits tarmac, and shoots again.
     * 
     * @param driveSubsystem
     * @param catapultSubsystem
     * @param intakePistonSubsystem
     * @param intakeMotorSubsystem
     * @param visionCameraSubsystem
     */
    public ShootMoveShoot(
        DriveSubsystem driveSubsystem,
        IntakePistonSubsystem intakePistonSubsystem, IntakeMotorSubsystem intakeMotorSubsystem,
        VisionCameraSubsystem visionCameraSubsystem
    ) {

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\

        // Exits the tarmac
        Trajectory t_exitTarmac = PathPlanner.loadPath("ShootMoveShoot", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_exitTarmac);

        //----- RAMSETE COMMANDS -----\\
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

        addCommands(
            new WaitCommand(0.5),
            new ResetAutonomousCommand(t_exitTarmac.getInitialPose(), driveSubsystem),
            new ParallelRaceGroup(
                new AutonomousAimCommand(visionCameraSubsystem, driveSubsystem),
                new WaitCommand(3)
            ),
            new ParallelRaceGroup(
                new EngageIntakePistonsCommand(intakePistonSubsystem),
                new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
                r_exitTarmac
            ),
            new ParallelRaceGroup(
                new AutonomousAimCommand(visionCameraSubsystem, driveSubsystem),
                new WaitCommand(3)
            )
        );

    } // End of Constructor
} // End of Class