package frc.robot.commands.autocommands.paths;

import com.pathplanner.lib.PathPlanner;

import frc.robot.commands.BallHolderCommand;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.OpenBallHolderCommand;
import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.CatapultCommand.CatapultPower;
import frc.robot.commands.autocommands.AutonomousAimCommand;
import frc.robot.commands.autocommands.ResetAutonomousCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;

//  -------- PATH DESCRIPTION -------- \\
//  Backs off tarmac picks up a ball and shoots two balls

public class TwoBallAuto extends PathPlannerSequentialCommandGroupUtility {

    // TO-DO comment this section
    private final double KMAXSPEED = 1.0;
    private final double KMAXACCELERATION = 1.0;
    private final double KRAMSETEB = 2;
    private final double KRAMSETEZETA = 0.7;
    private final DifferentialDriveOdometry m_odometry;

    /**
     * Default path constructor
     * 
     * @param dSubsystem
     */
    public TwoBallAuto(DriveSubsystem dSubsystem,
            IntakePistonSubsystem intakePistonSubsystem,
            IntakeMotorSubsystem intakeMotorSubsystem,
            VisionCameraSubsystem visionCameraSubsystem,
            CatapultSubsystem catapultSubsystem) {

        // initializing gyro for pose2d
        m_odometry = dSubsystem.getOdometry();

        // -------- Trajectories -------- \\

        // Generates a trajectory
        Trajectory trajectory1 = PathPlanner.loadPath("TwoBallDefense1", KMAXSPEED, KMAXACCELERATION);
        Trajectory trajectory2 = PathPlanner.loadPath("TwoBallDefense2", KMAXSPEED, KMAXACCELERATION);

        this.addTrajectory(trajectory1);
        this.addTrajectory(trajectory2);

        SmartDashboard.putString("Pos1", trajectory1.getInitialPose().toString());
        // -------- RAMSETE Commands -------- \\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        SmartDashboard.putString("current Gyro Position", m_odometry.getPoseMeters().toString());
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
        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends
        addCommands(
                new InstantCommand(catapultSubsystem::setShortShot),
                new ResetAutonomousCommand(trajectory1.getInitialPose(), dSubsystem),
                new ParallelRaceGroup(
                        new EngageIntakePistonsCommand(intakePistonSubsystem),
                        new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
                        ramseteCommand1),
                new StopDrive(dSubsystem),
                new ParallelRaceGroup(
                        new AutonomousAimCommand(visionCameraSubsystem, dSubsystem),
                        new WaitCommand(1)),
                new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons)
                        .withTimeout(CatapultSubsystem.SHOOT_TIMEOUT),

                
                //new WaitCommand(1.25),//idea 2 sec
                new WaitCommand(1.0),
                new ParallelRaceGroup(                                
                        new BallHolderCommand(catapultSubsystem, true),
                        new WaitCommand(2)),
                new WaitCommand(2.0),
                new OpenBallHolderCommand(catapultSubsystem).withTimeout(0.5),

                new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons)
                        .withTimeout(CatapultSubsystem.SHOOT_TIMEOUT),
                new WaitCommand(0.25),
                new ParallelRaceGroup(
                        new EngageIntakePistonsCommand(intakePistonSubsystem),
                        new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
                        ramseteCommand2),
                new StopDrive(dSubsystem),
                new WaitCommand(2),
                new CatapultCommand(catapultSubsystem, CatapultPower.LargePistons)
                        .withTimeout(CatapultSubsystem.SHOOT_TIMEOUT));

    } // End of Constructor
} // End of Class