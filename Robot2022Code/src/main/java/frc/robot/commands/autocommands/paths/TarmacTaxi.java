//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.autocommands.AutoBase;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.subsystems.DriveSubsystem;

//----- CLASS -----\\
/**
 * <h3>TarmacTaxi</h3>
 * 
 * Moves forward 60 inches to exit the tarmac. Consequently, autonomous taxi points are gained.
 */
public class TarmacTaxi extends AutoBase {

    //----- CONSTANTS -----\\

    // Movement Control
    private final static double MAX_SPEED = 0.5;
    private final static double MAX_ACCELERATION = 2.5;

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
        super(driveSubsystem, PathPlanner.loadPath("TaxiOneBall", MAX_SPEED, MAX_ACCELERATION));

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\

        // Forward 60 inches;
        this.addTrajectory(super.m_initialTrajectory);

        //----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command r_forwardSixtyInches = new Ramsete930Command(
            super.m_initialTrajectory,
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