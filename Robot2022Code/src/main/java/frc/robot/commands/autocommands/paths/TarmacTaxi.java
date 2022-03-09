//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import java.util.List;

import frc.robot.commands.Ramsete930Command;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;

//----- CLASS -----\\
/**
 * <h3>TarmacTaxi</h3>
 * 
 * Moves forward 60 inches to exit the tarmac. Consequently, autonomous taxi points are gained.
 */
public class TarmacTaxi extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    private final double MAX_SPEED = 0.5;
    private final double MAX_ACCELERATION = 2.5;

    // Ramsete Controller Parameters
    // private final double RAMSETE_B = 2;
    // private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>TarmacTaxi</h3>
     * 
     * Moves forward 60 inches to exit the tarmac. Consequently, autonomous taxi points are gained.
     * 
     * @param driveSubsystem
     */
    public TarmacTaxi(DriveSubsystem driveSubsystem) {

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- CONFIGURATION -----\\

        // Configurate the values of all trajectories for max velocity and acceleration
        TrajectoryConfig config = new TrajectoryConfig(
                MAX_SPEED,
                MAX_ACCELERATION
        )
                        // Add kinematics to ensure max speed is actually obeyed
                        // -- setEndVelocity stops the auto path at the end
                        .setKinematics(driveSubsystem.getKinematics()).setEndVelocity(0.0)
                        // Apply the voltage constraint
                        .addConstraint(driveSubsystem.getVoltageContraint());

        //----- TRAJECTORIES -----\\

        // Forward 60 inches
        Trajectory t_fowardSixtyInches = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                // Midpoints
                ),
                // End 5 feet infront of initiation line
                new Pose2d(Units.inchesToMeters(60.0), Units.inchesToMeters(0), new Rotation2d(0)),
                // Pass config
                config

        );
        this.addTrajectory(t_fowardSixtyInches);

        //----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_forwardSixtyInches = new Ramsete930Command(
            t_fowardSixtyInches,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\

        addCommands(
            r_forwardSixtyInches
        );

    } // End of Constructor
} // End of Class