package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import edu.wpi.first.math.controller.RamseteController;
import frc.robot.utilities.DifferentialDriveOdometry930;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

public class CompPath1 extends PathPlannerSequentialCommandGroupUtility {

    //  TO-DO comment this section
    private final double KMAXSPEED = 3.5;
    private final double KMAXACCELERATION = 3;
    private final double KRAMSETEB = 2;
    private final double KRAMSETEZETA = 0.7;
    private final DifferentialDriveOdometry930 m_odometry;

    /**
     * Default path constructor
     * 
     * @param dSubsystem
     */
    public CompPath1(DriveSubsystem dSubsystem, IntakePistonSubsystem intakePistonSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) { 

        //  initializing gyro for pose2d
        m_odometry = dSubsystem.getOdometry();

        // -------- Trajectories -------- \\

        // Generates a trajectory
        Trajectory trajectory1 = PathPlanner.loadPath("CompPath1pt1", 1.5, 2.5);

        this.addTrajectory(trajectory1);

        SmartDashboard.putString("Pos1",trajectory1.getInitialPose().toString());

        //Trajectory trajectory2 = PathPlanner.loadPath("CompPath1pt2", KMAXSPEED, KMAXACCELERATION);

        //this.addTrajectory(trajectory2);

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

        addCommands(new ResetOdometryCommand(trajectory1.getInitialPose(), dSubsystem),/*new EngageIntakePistonsCommand(intakePistonSubsystem), new RunIntakeMotorsCommand(intakeMotorSubsystem, false),*/ ramseteCommand1, new StopDrive(dSubsystem));

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