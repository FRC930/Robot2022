package frc.robot.commands.autocommands;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.utilities.GyroUtility;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.subsystems.DriveSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

public class ResetAutonomousCommand extends CommandBase {

    //  TO-DO comment this section
    private final DifferentialDriveOdometry m_odometry;
    private final DriveSubsystem m_dSubsystem;
    private final Pose2d m_startingPose;

    /**
     * Default path constructor
     * 
     * @param dSubsystem
     */
    public ResetAutonomousCommand(Pose2d startingPose, DriveSubsystem dSubsystem) { 

        //  initializing gyro for pose2d
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
    m_odometry.resetPosition(m_startingPose, new Rotation2d(Math.toRadians(GyroUtility.getInstance().getGyro().getFusedHeading())));
    m_dSubsystem.resetEncoders();
    //System.out.printf("Current Position x", m_startingPose.toString());
    
  }
  
   /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
} // End of Class