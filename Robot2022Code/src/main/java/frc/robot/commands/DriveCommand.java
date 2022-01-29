package frc.robot.commands;

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
public class DriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private VisionCameraSubsystem reflectiveCameraSubsystem;
    private VisionCameraSubsystem ballCameraSubsystem;
    private XboxController driverController;

    private final double JOYSTICK_DEADBAND = 0.15;

    private DoubleSupplier driveStick;
    private DoubleSupplier rotationStick;

    private PIDController turnController = new PIDController(1, 0, 0);

    /**
     * Initializes a new {@link frc.robot.commands.DriveCommand DriveCommand} with
     * the passed variables
     * 
     * @param dSubsystem       the drive subsystem to control
     * @param eSubsystem       where to get the pigeon for odometry
     * @param reflectSubsystem the camera subsystem to use to autmatically aim
     * @param dController      the driver's controller
     */
    public DriveCommand(DriveSubsystem dSubsystem, EndgameMotorSubsystem eSubsystem,
            VisionCameraSubsystem reflectSubsystem, VisionCameraSubsystem ballCamera, XboxController dController) {
        driveSubsystem = dSubsystem;
        reflectiveCameraSubsystem = reflectSubsystem;
        ballCameraSubsystem = ballCamera;
        driverController = dController;

        driveStick = () -> -deadbandCube(driverController.getLeftY()) * DriveSubsystem.kMaxSpeed;
        rotationStick = () -> -deadbandCube(driverController.getRightX()) * DriveSubsystem.kMaxAngularSpeed;

        // We are not adding endgame motor subsystem as a requirement because we are not
        // using the subsystem in the command at all
        addRequirements(driveSubsystem, reflectiveCameraSubsystem);
    }

    @Override
    public void execute() {
        // No need to modify drive speed axis
        double xStick = driveStick.getAsDouble();
        double rotationSpeed;

        // If the right stick button is pressed
        if (driverController.getRightStickButton()) {
            // Get the frame with annotations from PhotonVision
            PhotonPipelineResult result;
            if (DriveCameraUtility.getInstance().getCameraState() == CameraStates.BALL) {
                if (DriveCameraUtility.getInstance().getBallColor() == BallColor.BLUE) {
                    ballCameraSubsystem.getVisionCamera().setPipelineIndex(1);
                    result = ballCameraSubsystem.getVisionCamera().getLatestResult();
                } else {
                    ballCameraSubsystem.getVisionCamera().setPipelineIndex(0);
                    result = ballCameraSubsystem.getVisionCamera().getLatestResult();
                }
            } else {
                result = reflectiveCameraSubsystem.getVisionCamera().getLatestResult();
            }
            
            // Check if the camera sees any targets
            if (result.hasTargets()) {
                

                // Use the turnController PID controller to orient the robot toward the goal
                // May want to use a smoothing function to try and reduce the effects of wayward
                // vision targets
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
                // Make sure we are not passing in weird values for the imaginary stick
                rotationSpeed = MathUtil.clamp(rotationSpeed, -1, 1);

                //if degrees is equal to -1 to 1, vibrate the controller
                double xDegreeOffset = result.getBestTarget().getYaw();
                if(xDegreeOffset > -1 && xDegreeOffset < 1) {
                    driverController.setRumble(RumbleType.kLeftRumble, 1);
                }
            } else {
                // Since there are no targets, set rotation to zero
                // TODO: don't have this in a println
                System.out.println("NO TARGETS: AUTOVISION CAN\'T PERFORM");
                rotationSpeed = 0;
            }
        } else {
            // Just get the right stick horizontal axis
            rotationSpeed = rotationStick.getAsDouble();
        }

        // Get the wheel speeds from the stick values
        DifferentialDriveWheelSpeeds wheelSpeeds = driveSubsystem.getWheelSpeeds(xStick,
                rotationSpeed);

        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.testingTab, ShuffleboardKeys.LEFT_SPEED,
                new ShuffleBoardData<Double>(wheelSpeeds.leftMetersPerSecond));
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.testingTab,
                ShuffleboardKeys.RIGHT_SPEED,
                new ShuffleBoardData<Double>(wheelSpeeds.rightMetersPerSecond));

        driveSubsystem.setVoltages(
                // Use the speed to voltage method in the drive subsystem
                driveSubsystem.speedToVoltage(
                        // Calculate feedforward with the feedforward controller in drive subsystem
                        driveSubsystem.calculateLeftFeedforward(wheelSpeeds.leftMetersPerSecond)),
                // Again, speed to voltage
                driveSubsystem.speedToVoltage(
                        // Same deal here, feedforward using the helper method
                        driveSubsystem.calculateRightFeedforward(wheelSpeeds.rightMetersPerSecond)));
        // Update the differential drive odometry
        // Might be possible to remove it from the default teleop command
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0);
    }

    private double deadbandCube(double stickPos) {
        // Check that the value is stick value is outside the deadband
        if (stickPos < JOYSTICK_DEADBAND && stickPos > -JOYSTICK_DEADBAND) {
            // HAHAHA no bread for you
            return 0;
        } else {
            // Here, you can have this bread*bread*bread (cubed)
            // Cube the stick input to result in a smoother transition from low to high
            // speeds
            return Math.pow(stickPos, 3);
        }
    }
}
