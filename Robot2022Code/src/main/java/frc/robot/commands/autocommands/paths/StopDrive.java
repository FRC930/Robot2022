package frc.robot.commands.autocommands.paths;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.ShifterUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.DriveCameraUtility.BallColor;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

import static frc.robot.utilities.DriveCameraUtility.CameraStates;

/**
 * <h3>DriveCommand</h3>
 * 
 * DriveCommand takes care of driving during teleop. When the right stick button
 * is pressed, the robot will automatically take over aiming using the
 * PhotonCamera passed in the constructor
 * 
 * @author Alexander Taylor, Jack LaFreniere, and Anthony Witt
 * @since 22 January 2022
 * @version 1.0
 */
public class StopDrive extends CommandBase {
    private DriveSubsystem driveSubsystem;
    
    /**
     * Initializes a new {@link frc.robot.commands.DriveCommand DriveCommand} with
     * the passed variables
     * 
     * @param dSubsystem       the drive subsystem to control
     * @param eSubsystem       where to get the pigeon for odometry
     * @param reflectSubsystem the camera subsystem to use to autmatically aim
     * @param dController      the driver's controller
     */
    public StopDrive(
            DriveSubsystem dSubsystem) {
        driveSubsystem = dSubsystem;
        // We are not adding endgame motor subsystem as a requirement because we are not
        // using the subsystem in the command at all
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        DifferentialDriveWheelSpeeds wheelSpeeds = driveSubsystem.getWheelSpeeds(0.0, 0.0);
        driveSubsystem.setVoltages(driveSubsystem.speedToVoltage(wheelSpeeds.leftMetersPerSecond), driveSubsystem.speedToVoltage(wheelSpeeds.rightMetersPerSecond));
    }

    @Override
    public boolean isFinished() {
      return true;
    } // End of isFinished()
}
