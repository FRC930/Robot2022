package frc.robot.commands.autocommands.paths;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// -------- PATH DESCRIPTION -------- \\
// Mid Field - Move off intiation & Initial 3

public class DefaultAutoPath extends SequentialCommandGroup {
    /**
    * Creates a new Autonomous.
    */
    private final double KSVOLTS = 0.411;
    private final double KVVOLT = 0.227;
    public final double KAVOLT = 0.0249;
    public final double KMAXSPEED = 3.5;
    public final double KMAXACCELERATION = 3;
    public final double KRAMSETEB = 2;
    public final double KRAMSETEZETA = 0.7;
    public final double KTRACKWIDTH = 0.381;
    public final DifferentialDriveKinematics KDRIVEKINEMATICS = new DifferentialDriveKinematics(KTRACKWIDTH);
    public final double KPDRIVEVEL =  0.693;

    public DefaultAutoPath(DriveSubsystem dSubsystem) {
    
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(KSVOLTS,
            KVVOLT,
            KAVOLT),
            KDRIVEKINEMATICS,10);
    
    // Configurate the values of all trajectories for max velocity and acceleration
    TrajectoryConfig config =
      new TrajectoryConfig(KMAXSPEED,
      KMAXACCELERATION)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(KDRIVEKINEMATICS)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

    // -------- Trajectories -------- \\

    // Generates a trajectory 
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        // Start 
        new Pose2d(inchesToMeters(0), inchesToMeters(0), new Rotation2d(0)),
        List.of(
            // Midpoints
        ),
        // End 5 feet infront of initiation line
        new Pose2d(inchesToMeters(60.0), inchesToMeters(0), new Rotation2d(0)),
        // Pass config
        config

    );

    // -------- RAMSETE Commands -------- \\
    // Creates a command that can be added to the command scheduler in the sequential command
    
    // Creates RAMSETE Command for first trajectory
    /**
    RamseteCommand ramseteCommand1 = new RamseteCommand(
        trajectory1,
        dSubsystem::getPose,
        new RamseteController(KRAMSETEB, KRAMSETEZETA),
        new SimpleMotorFeedforward(KSVOLTS,
                                   KVVOLT,
                                   KAVOLT),
        KDRIVEKINEMATICS,
        dSubsystem::getWheelSpeeds,
        new PIDController(KPDRIVEVEL, 0, 0),
        new PIDController(KPDRIVEVEL, 0, 0),
        // RamseteCommand passes volts to the callback
        dSubsystem::tankDriveVolts,0
        dSubsystem
    );
    */
    
    /*
    Path Description:
    -----------------
        Robot has 3 power cells set on top of the robot
        Robot Shoots 3 power cells and moves off initiation linE
    */

    } // End of Constructor

    // Method to convert distances
    public double inchesToMeters(double inches) {
        double meters = inches / 39.37;
        return meters;
    }

} // End of Class