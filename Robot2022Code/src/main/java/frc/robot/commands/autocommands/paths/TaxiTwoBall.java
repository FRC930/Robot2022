//----- IMPORTS -----\\

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

//----- CLASS -----\\
/**
 * <h3>TaxiTwoBall</h3>
 * 
 * Exits the tarmac, intakes, and shoots.
 */
public class TaxiTwoBall extends PathPlannerSequentialCommandGroupUtility {

    //----- CONSTANTS -----\\

    // Movement Control
    private final double MAX_SPEED = 1.0;
    private final double MAX_ACCELERATION = 1.0;

    // Ramsete Controller Parameters
    private final double RAMSETE_B = 2;
    private final double RAMSETE_ZETA = 0.7;

    //----- ODOMETRY -----\\

    private final DifferentialDriveOdometry m_odometry;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>TaxiTwoBall</h3>
     * 
     * Exits the tarmac, intakes, and shoots.
     * 
     * @param driveSubsystem
     * @param intakePistonSubsystem
     * @param intakeMotorSubsystem
     * @param visionCameraSubsystem
     * @param catapultSubsystem
     */
    public TaxiTwoBall(
        DriveSubsystem driveSubsystem,
        IntakePistonSubsystem intakePistonSubsystem,
        IntakeMotorSubsystem intakeMotorSubsystem,
        VisionCameraSubsystem visionCameraSubsystem,
        CatapultSubsystem catapultSubsystem
    ) {

        // initializing gyro for pose2d
        m_odometry = driveSubsystem.getOdometry();

        //----- TRAJECTORIES -----\\

        // Robot exits the tarmac, intakes, and shoots
        Trajectory t_exitTarmac = PathPlanner.loadPath("TaxiTwoBall", MAX_SPEED, MAX_ACCELERATION);

        this.addTrajectory(t_exitTarmac);

        SmartDashboard.putString("Pos1", t_exitTarmac.getInitialPose().toString());
        SmartDashboard.putString("current Gyro Position", m_odometry.getPoseMeters().toString());

        //----- RAMSETE COMMMANDS -----\\
        // Creates a command that can be added to the command scheduler in the
        // sequential command

        Ramsete930Command r_exitTarmac = new Ramsete930Command(
            t_exitTarmac,
            () -> m_odometry.getPoseMeters(),
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            driveSubsystem.getKinematics(),
            driveSubsystem::getWheelSpeeds,
            (Double leftVoltage, Double rightVoltage) -> driveSubsystem.setVoltages(leftVoltage, rightVoltage),
            driveSubsystem
        );

        //----- AUTO SEQUENCE -----\\
        // Parallel Race Group ends these commands because they dont end, they end when
        // the wait command ends

        addCommands(
            new InstantCommand(catapultSubsystem::setShortShot),
            new ResetAutonomousCommand(t_exitTarmac.getInitialPose(), driveSubsystem),
            new ParallelRaceGroup(
                new EngageIntakePistonsCommand(intakePistonSubsystem),
                new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
                r_exitTarmac
            ),
            new StopDrive(driveSubsystem),
            new ParallelRaceGroup(
                new AutonomousAimCommand(visionCameraSubsystem, driveSubsystem),
                new WaitCommand(1)
            ),
            new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons)
                .withTimeout(CatapultSubsystem.SHOOT_TIMEOUT),

            
            //new WaitCommand(1.25),//idea 2 sec
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