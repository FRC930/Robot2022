package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.CatapultCommand;
import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.CatapultCommand.CatapultPower;
import frc.robot.commands.autocommands.AutonomousAimCommand;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.autovisioncommands.HubAimingCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Moves forward 60 inches

public class TerminalPickup extends PathPlannerSequentialCommandGroupUtility {

    //  TO-DO comment this section
    private final double KMAXSPEED = 1.5;
    private final double KMAXACCELERATION = 3;
    private final double KRAMSETEB = 2;
    private final double KRAMSETEZETA = 0.7;
    private final DifferentialDriveOdometry m_odometry;

    /**
     * Default path constructor
     * @author Ed Pilon, Hussain Mehdi, and Caden DeGlopper
     * @param dSubsystem
     */
    public TerminalPickup(
        DriveSubsystem dSubsystem, 
        IntakePistonSubsystem intakePistonSubsystem,
        IntakeMotorSubsystem intakeMotorSubsystem,
        VisionCameraSubsystem visionCameraSubsystem,
        CatapultSubsystem catapultSubsystem) { 

        //  initializing gyro for pose2d
        m_odometry = dSubsystem.getOdometry();

        // -------- Trajectories -------- \\

        // Generates a trajectory
        Trajectory trajectory1 = PathPlanner.loadPath("TerminalPickup1", KMAXSPEED, KMAXACCELERATION);

        this.addTrajectory(trajectory1);

        Trajectory trajectory2 = PathPlanner.loadPath("TerminalPickup2", KMAXSPEED, KMAXACCELERATION);

        this.addTrajectory(trajectory2);

        Trajectory trajectory3 = PathPlanner.loadPath("TerminalPickup3", KMAXSPEED, KMAXACCELERATION);

        this.addTrajectory(trajectory3);
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

        Ramsete930Command ramseteCommand2 = new Ramsete930Command(
                trajectory2,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(KRAMSETEB, KRAMSETEZETA),
                dSubsystem.getKinematics(),
                dSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> dSubsystem.setVoltages(leftVoltage, rightVoltage),
                dSubsystem);

        Ramsete930Command ramseteCommand3 = new Ramsete930Command(
                trajectory3,
                () -> m_odometry.getPoseMeters(),
                new RamseteController(KRAMSETEB, KRAMSETEZETA),
                dSubsystem.getKinematics(),
                dSubsystem::getWheelSpeeds,
                (Double leftVoltage, Double rightVoltage) -> dSubsystem.setVoltages(leftVoltage, rightVoltage),
                dSubsystem);
         
                

      // Parallel Race Group ends these commands because they dont end, they end when the wait command ends
        // Sets catapult shot to short
        // reset the encoders and odometry to where our path starts
        // starts the path
        // engages the intake piston runs them at the same time
        // it stops driving.
        addCommands(new ParallelRaceGroup(new CatapultCommand(catapultSubsystem, CatapultPower.SetShortShot), new WaitCommand(0.5)),
        new ResetAutonomousCommand(trajectory1.getInitialPose(), dSubsystem),
        new ParallelRaceGroup(new EngageIntakePistonsCommand(intakePistonSubsystem), new RunIntakeMotorsCommand(intakeMotorSubsystem, false), ramseteCommand1),
        new StopDrive(dSubsystem), 
        // aim then shoot 
        // moves to next position 
        // while engaging then running the intake pistons,
        // waits 2.5 seconds, 
        // runs the intake pistons 
        // moves to position 3
        new AutonomousAimCommand(visionCameraSubsystem, dSubsystem),
        new ParallelRaceGroup(new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons), new WaitCommand(1)),

        new ParallelRaceGroup(new ParallelCommandGroup(new EngageIntakePistonsCommand(intakePistonSubsystem), new RunIntakeMotorsCommand(intakeMotorSubsystem, false)), 
                              new SequentialCommandGroup(ramseteCommand2, new WaitCommand(2.5), ramseteCommand3)),
        new StopDrive(dSubsystem),
        // This segment of the path aims 
        //shoots the catapult
        new ParallelRaceGroup(new HubAimingCommand(visionCameraSubsystem, dSubsystem), new WaitCommand(1)),
        new ParallelRaceGroup(new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons), new WaitCommand(1))
        );
    } // End of Constructor
} // End of Class