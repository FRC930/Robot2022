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
    public DriveCommand(
        DriveSubsystem dSubsystem, 
        VisionCameraSubsystem reflectSubsystem, 
        VisionCameraSubsystem ballCamera, 
        XboxController dController
    ) {
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

        // Result from the pipeline
        PhotonPipelineResult result = null;

        // If X button is pressed, aim towards reflective tape
        // Else if B button is pressed, aim towards position of ball
        // Else drive with right joystick (manual)
        if (driverController.getXButtonPressed()) {

            // Get latest result from the reflective camera
            result = reflectiveCameraSubsystem.getVisionCamera().getLatestResult();

            // Gets rotation speed required to face the reflective tape
            rotationSpeed = rotateTowardsTarget(result);

        } else if (driverController.getBButtonPressed()) {

            // If the ball is red, set the pipeline to red and get the latest result
            // Else (the ball is blue), set the pipeline to blue and get the latest result
            if (DriveCameraUtility.getInstance().getBallColor() == BallColor.RED) {

                // Sets the pipeline to red
                ballCameraSubsystem.getVisionCamera().setPipelineIndex(0);

                // Gets latest result from the ball camera
                result = ballCameraSubsystem.getVisionCamera().getLatestResult();

            } else {

                // Sets the pipeline to blue
                ballCameraSubsystem.getVisionCamera().setPipelineIndex(1);

                // Gets latest result from the ball camera
                result = ballCameraSubsystem.getVisionCamera().getLatestResult();

            }

            // Gets rotation speed required to face towards the targetted ball
            rotationSpeed = rotateTowardsTarget(result);

        } else {

            // Just get the right stick horizontal axis
            rotationSpeed = rotationStick.getAsDouble();

        }

        // Checking a result exists and X or B is pressed
        // If true, it will rumble based on a Yaw range
        // Else, no rumble
        if(result != null && (driverController.getBButtonPressed() || driverController.getXButtonPressed())) {
            // Checking if the result is within one degree of the target
            // If true, rumble. If not within the range, no rumble.
            if(result.getBestTarget().getYaw() > -1 && result.getBestTarget().getYaw() < 1) {
                driverController.setRumble(RumbleType.kRightRumble, 1);
            } else {
                driverController.setRumble(RumbleType.kRightRumble, 0);
            }
        } else {
            driverController.setRumble(RumbleType.kRightRumble, 0);
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

    private double rotateTowardsTarget(PhotonPipelineResult result) {

        double rotationSpeed = 0;

        // Check if the camera sees any targets
        if (result.hasTargets()) {

            // Use the turnController PID controller to orient the robot toward the goal
            // May want to use a smoothing function to try and reduce the effects of wayward
            // vision targets
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            // Make sure we are not passing in weird values for the imaginary stick
            rotationSpeed = MathUtil.clamp(rotationSpeed, -1, 1);

            // if degrees is equal to -1 to 1, vibrate the controller
            double xDegreeOffset = result.getBestTarget().getYaw();
            if (xDegreeOffset > -1 && xDegreeOffset < 1) {
                driverController.setRumble(RumbleType.kLeftRumble, 1);
            }
        } else {
            // Since there are no targets, set rotation to zero
            // TODO: don't have this in a println
            System.out.println("NO TARGETS: AUTOVISION CAN\'T PERFORM");
            rotationSpeed = 0;
        }

        return rotationSpeed;
    }
}
