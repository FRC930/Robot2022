package frc.robot.commands.autocommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.utilities.GyroUtility;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

/**
 * <h3>ResetAutonomousCommand</h3>
 * 
 * Resets the gyro and resemtog
 * 
 * @author Ed Pilon, Hussain Mehdi, and Caden DeGlopper
 */
public class ResetAutonomousCommand extends CommandBase {

    // TO-DO comment this section
    private final DifferentialDriveOdometry m_odometry;
    private final DriveSubsystem m_dSubsystem;
    private final Pose2d m_startingPose;

    /**
     * <h3>ResetAutonomousCommand</h3>
     * 
     * @param startingPose
     * @param dSubsystem
     */
    public ResetAutonomousCommand(Pose2d startingPose, DriveSubsystem dSubsystem) {

        // initializing gyro for pose2d
        m_dSubsystem = dSubsystem;
        m_odometry = m_dSubsystem.getOdometry();
        m_startingPose = startingPose;

        addRequirements(m_dSubsystem);

    } // End of Constructor

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_odometry.resetPosition(m_startingPose,
                new Rotation2d(Math.toRadians(GyroUtility.getInstance().getGyro().getYaw())));
        m_dSubsystem.resetEncoders();
        // System.out.printf("Current Position x", m_startingPose.toString());

    }

    /**
     * Returns true when the command should end.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
} // End of Class