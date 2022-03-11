//----- IMPORTS -----\\

package frc.robot.commands.autocommands.paths;

import java.util.List;

import frc.robot.commands.BallHolderCommand;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.OpenBallHolderCommand;
import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.CatapultCommand.CatapultPower;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;

//----- CLASS -----\\
/**
 * <h3>TaxiOneBall</h3>
 * 
 * Exits the tarmac and shoots.
 */
public class TaxiOneBall extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    private final double MAX_SPEED = 0.5; // DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH
    private final double MAX_ACCELERATION = 2.5;

    // Ramsete Controller Parameters
    // private final double RAMSETE_B = 2;
    // private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    /**
     * <h3>TaxiOneBall</h3>
     * 
     * Exits the tarmac and shoots.
     * 
     * @param driveSubsystem
     * @param catapultSubsystem
     */
    public TaxiOneBall(DriveSubsystem driveSubsystem, CatapultSubsystem catapultSubsystem) {

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

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

        // Generates a trajectory
        Trajectory t_exitTarmac = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
            List.of(
            // Midpoints
            ),
            // End 5 feet infront of initiation line
            new Pose2d(Units.inchesToMeters(60.0), Units.inchesToMeters(0), new Rotation2d(0)),
            // Pass config
            config
        );
        this.addTrajectory(t_exitTarmac);

        //----- RAMSETE COMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        Ramsete930Command r_exitTarmac = new Ramsete930Command(
            t_exitTarmac,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(), // new RamseteController(RAMSETE_B, RAMSETE_ZETA)
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage,
                    rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\

        addCommands(
            r_exitTarmac,

            new WaitCommand(1.0),
            new ParallelRaceGroup(                                
                new BallHolderCommand(catapultSubsystem, true),
                new WaitCommand(2)
            ),
            new WaitCommand(2.0),
            new OpenBallHolderCommand(catapultSubsystem).withTimeout(0.5),

            new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons)
                .withTimeout(CatapultSubsystem.SHOOT_TIMEOUT)
        );

    } // End of Constructor
} // End of Class