package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.CatapultCommand;
import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.CatapultCommand.CatapultPower;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

public class ShootMoveShoot extends PathPlannerSequentialCommandGroupUtility {

    //  TO-DO comment this section
    private final double KMAXSPEED = 0.5;
    private final double KMAXACCELERATION = 0.5;
    private final double KRAMSETEB = 2;
    private final double KRAMSETEZETA = 0.7;
    private final DifferentialDriveOdometry m_odometry;

    /**
     * Default path constructor
     * 
     * @param dSubsystem
     */
    public ShootMoveShoot(DriveSubsystem dSubsystem, CatapultSubsystem catapultSubsystem, IntakePistonSubsystem intakePistonSubsystem, IntakeMotorSubsystem intakeMotorSubsystem) { 

        //  initializing gyro for pose2d
        m_odometry = dSubsystem.getOdometry();

        // -------- Trajectories -------- \\

        // Generates a trajectory
        Trajectory trajectory1 = PathPlanner.loadPath("ShootMoveShoot", KMAXSPEED, KMAXACCELERATION);

        this.addTrajectory(trajectory1);
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

        addCommands(new ParallelRaceGroup(new CatapultCommand(catapultSubsystem, CatapultPower.SetShortShot), new WaitCommand(0.5)),
        new ResetAutonomousCommand(trajectory1.getInitialPose(), dSubsystem),
        new ParallelRaceGroup(new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons), new WaitCommand(1)),
        new ParallelRaceGroup(new EngageIntakePistonsCommand(intakePistonSubsystem), new RunIntakeMotorsCommand(intakeMotorSubsystem, false), ramseteCommand1),
        new ParallelRaceGroup(new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons), new WaitCommand(1)));

    } // End of Constructor
} // End of Class