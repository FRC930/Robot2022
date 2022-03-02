package frc.robot.commands.autocommands.paths;

import java.util.List;

import frc.robot.commands.CatapultCommand;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

public class DefaultShoot extends PathPlannerSequentialCommandGroupUtility {

    // TO-DO comment this section
    private final double KMAXSPEED = 0.5; //DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH
    private final double KMAXACCELERATION = 2.5;
    //private final double KRAMSETEB = 2;
    //private final double KRAMSETEZETA = 0.7;
    private final DifferentialDriveOdometry m_odometry;

    /**
     * Default path constructor
     * 
     * @param dSubsystem
     */
    public DefaultShoot(DriveSubsystem dSubsystem, CatapultSubsystem catapultSubsystem) {

        // initializing gyro for pose2d
        m_odometry = dSubsystem.getOdometry();

        // Configurate the values of all trajectories for max velocity and acceleration
        TrajectoryConfig config = new TrajectoryConfig(KMAXSPEED,
                KMAXACCELERATION)
                        // Add kinematics to ensure max speed is actually obeyed
                        // -- setEndVelocity stops the auto path at the end
                        .setKinematics(dSubsystem.getKinematics()).setEndVelocity(0.0)
                        // Apply the voltage constraint
                        .addConstraint(dSubsystem.getVoltageContraint());

        // -------- Trajectories -------- \\

        // Generates a trajectory
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                // Midpoints
                ),
                // End 5 feet infront of initiation line
                new Pose2d(Units.inchesToMeters(60.0), Units.inchesToMeters(0), new Rotation2d(0)),
                // Pass config
                config

        );
        this.addTrajectory(trajectory1);

        // -------- RAMSETE Commands -------- \\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command ramseteCommand1 = new Ramsete930Command(
                trajectory1,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(),//new RamseteController(KRAMSETEB, KRAMSETEZETA)
                dSubsystem.getKinematics(),
                dSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> dSubsystem.setVoltages(leftVoltage, rightVoltage),
                dSubsystem);
        //TODO: ADD BALL HOLDER COMMAND
        addCommands(ramseteCommand1, new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons).withTimeout(CatapultSubsystem.SHOOT_TIMEOUT));

    } // End of Constructor
} // End of Class