//----- IMPORTS -----\\

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.DriveSubsystem;

//----- CLASS -----\\
/**
 * <h3>FiveBallAuto</h3>
 * 
 * Four ball auto. Starts near the center of the tarmac, intakes, shoots, moves
 * to terminal, intakes, moves back to tarmac, and shoots.
 */
public class AutoBase extends PathPlannerSequentialCommandGroupUtility {

    // ----- CONSTANTS -----\\

    protected Trajectory m_initialTrajectory;

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
    public AutoBase(DriveSubsystem driveSubsystem, Trajectory initialTrajectory) {
        m_initialTrajectory = initialTrajectory;
        // ----- AUTO SEQUENCE -----\\

        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends
        // Sets catapult shot to short
        // reset the encoders and odometry to where our path starts
        // starts the path
        // engages the intake piston runs them at the same time
        // it stops driving.
        addCommands(
            new ResetAutonomousCommand(m_initialTrajectory.getInitialPose(), driveSubsystem)
        );
    } // End of Constructor
} // End of Class