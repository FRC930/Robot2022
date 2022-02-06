package frc.robot.commands.autocommands.paths;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.Ramsete930Command;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

public class BottomBackShootCommand extends SequentialCommandGroup {

    //  TO-DO comment this section
    private final double KMAXSPEED = 3.5;
    private final double KMAXACCELERATION = 3;
    private final double KRAMSETEB = 2;
    private final double KRAMSETEZETA = 0.7;
    private final DifferentialDriveOdometry m_odometry;

    /**
     * Default path constructor
     * 
     * @param dSubsystem
     */
    public BottomBackShootCommand(DriveSubsystem dSubsystem) { 

        //  initializing gyro for pose2d
        m_odometry = dSubsystem.getOdometry();

        // -------- Trajectories -------- \\

        // Generates a trajectory
        Trajectory trajectory1 = PathPlanner.loadPath("BottomBackShoot", KMAXSPEED, KMAXACCELERATION);

        // -------- RAMSETE Commands -------- \\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        // Creates RAMSETE Command for first trajectory
        Ramsete930Command ramseteCommand1 = new Ramsete930Command(
                trajectory1,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(KRAMSETEB, KRAMSETEZETA),
                dSubsystem.getKinematics(),
                dSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> dSubsystem.setVoltages(leftVoltage, rightVoltage),
                dSubsystem);

        addCommands(ramseteCommand1);

    } // End of Constructor

    /**
     * Converts Inches into meters
     * 
     * @param inches
     * @return meters
     */
    public double inchesToMeters(double inches) {
        double meters = inches / 39.37;
        return meters;
    }

} // End of Class