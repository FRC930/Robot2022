//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.DriveSubsystem;

//----- CLASS -----\\
/**
 * <h3>TemplateCommand</h3>
 * 
 * A template for auto paths.
 */
public class TemplateCommand extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    private final double MAX_SPEED = 0.0;
    private final double MAX_ACCELERATION = 0.0;

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 0.0;
    private final double RAMSETE_ZETA = 0.0;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>TemplateCommand</h3>
     * 
     * A template for auto paths.
     * 
     * @param driveSubsystem
     * @param catapultSubsystem
     */
    public TemplateCommand(DriveSubsystem driveSubsystem) {

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\

        // Describe your trajectory here
        Trajectory t_nameYourTrajectoryHere = PathPlanner.loadPath("<ENTER PATHPLANNER PATH NAME HERE>", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_nameYourTrajectoryHere);

        //----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_nameYourTrajectoryHere = new Ramsete930Command(
            t_nameYourTrajectoryHere,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\

        addCommands(
            new ResetAutonomousCommand(t_nameYourTrajectoryHere.getInitialPose(), driveSubsystem),
            r_nameYourTrajectoryHere,
            new StopDrive(driveSubsystem)
        );

    } // End of Constructor
} // End of Class
