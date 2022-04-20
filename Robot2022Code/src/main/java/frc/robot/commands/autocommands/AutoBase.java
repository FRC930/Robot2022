//----- IMPORTS -----\\

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.DriveSubsystem;

//----- CLASS -----\\
/**
 * <h3>AutoBase</h3>
 * 
 * Adds commands to the start of the autonomous period before the path runs
 */
public class AutoBase extends PathPlannerSequentialCommandGroupUtility {

    // ----- CONSTANTS -----\\

    protected Trajectory m_initialTrajectory;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>AutoBase</h3>
     * 
     * Adds commands to the start of the autonomous period before the path runs
     * 
     * @param driveSubsystem
     * @param intakePistonSubsystem
     * @param intakeMotorSubsystem
     * @param visionCameraSubsystem
     * @param catapultSubsystem
     */
    public AutoBase(DriveSubsystem driveSubsystem, Trajectory initialTrajectory) {
        m_initialTrajectory = initialTrajectory;
        // ----- Commands -----\\
        addCommands(
                new ResetAutonomousCommand(m_initialTrajectory.getInitialPose(), driveSubsystem));
    } // End of Constructor
} // End of Class