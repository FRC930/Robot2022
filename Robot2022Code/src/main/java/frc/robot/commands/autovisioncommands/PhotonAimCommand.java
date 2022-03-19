//----- IMPORTS -----\\

package frc.robot.commands.autovisioncommands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PhotonVisionUtility;
import frc.robot.utilities.ShooterUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;
import frc.robot.utilities.VisionSmoothingStack;

//----- CLASS -----\\
/**
 * <h3>HubAimingCommand</h3>
 * 
 * Rotates the robot to aim at the cargo hub.
 */
public class PhotonAimCommand extends CommandBase {

    // ----- VARIABLES -----\\

    // Yaw offset allowance in degrees
    public static final double YAW_OFFSET = 3.0;

    // The height of the camera
    private final double CAMERA_HEIGHT_METERS = Units.feetToMeters(4);
    // The height of the hub
    private final double HUB_HEIGHT_METERS = Units.inchesToMeters(104);

    private final double HEIGHT_DIFFERENCE_METERS = HUB_HEIGHT_METERS - Units.inchesToMeters(46);

    // The pitch of the camera
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(20.0);

    private final double HUB_RANGE_METERS = Units.feetToMeters(6);

    private final double LINEAR_P = 0.0;
    private final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    private final double ANGULAR_P = 0.4;
    private final double ANGULAR_D = 0.01;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    protected PhotonCamera hubCamera = PhotonVisionUtility.getInstance().getHubTrackingCamera();
    private DriveSubsystem driveSubsystem;

    private VisionSmoothingStack smoothingStack = new VisionSmoothingStack(3);

    private XboxController driverController;
    private XboxController codriverController;

    // ----- CONSTRUCTORS -----\\

    /**
     * <h3>HubAimingCommand</h3>
     * 
     * Rotates the robot to aim at the cargo hub.
     * 
     * @param dSubsystem
     */
    public PhotonAimCommand(DriveSubsystem dSubsystem) {
        this(dSubsystem, null, null);
    }

    /**
     * <h3>HubAimingCommand</h3>
     * 
     * Rotates the robot to aim at the cargo hub.
     * 
     * @param dSubsystem
     * @param driverController
     * @param coDriverController
     */
    public PhotonAimCommand(DriveSubsystem dSubsystem, XboxController driverController,
            XboxController coDriverController) {

        driveSubsystem = dSubsystem;

        addRequirements(dSubsystem);
    }

    // ----- METHODS -----\\

    @Override
    public void initialize() {
        hubCamera.setLED(VisionLEDMode.kOn);

        driveSubsystem.setVoltages(0, 0);

        PhotonVisionUtility.getInstance()
                .setPiCamerPipeline(ShuffleboardUtility.getInstance().getSelectedPipelineChooser());
    }

    @Override
    public void execute() {
        // Variables to store our speeds
        double forwardSpeed;
        double rotationSpeed;

        // Data that we get from the camera
        var result = hubCamera.getLatestResult();

        // If an item is detected
        if (result.hasTargets()) {
            // Finds the best target and puts into the stack
            smoothingStack.addItem(result.getBestTarget());

            // Use the PhotonUtils library to calcluate the distance from the target
            double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, HUB_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS, Units.degreesToRadians(smoothingStack.getAveragePitch()));

            range = Math.sqrt(Math.pow(range, 2) - Math.pow(HEIGHT_DIFFERENCE_METERS, 2));

            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.DISTANCE_FROM_GOAL, new ShuffleBoardData<Double>(range));

            // Calculate shooter values
            ShooterUtility.setValuesToShuffleboard(range);

            // Use our forward PID controller to calculate how fast we want to go forward
            forwardSpeed = -forwardController.calculate(range, HUB_RANGE_METERS);

            // Use the turn PID controller to calculate how fast we want to turn
            rotationSpeed = turnController.calculate(smoothingStack.getAverageYaw(), 0);

            // Put if we are locked onto the target to the Shuffleboard
            if (Math.abs(smoothingStack.getAverageYaw()) < YAW_OFFSET) {
                ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                        ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(true));

                if (driverController != null && codriverController != null) {
                    driverController.setRumble(RumbleType.kLeftRumble, 1);
                    driverController.setRumble(RumbleType.kRightRumble, 1);
                    codriverController.setRumble(RumbleType.kLeftRumble, 1);
                    codriverController.setRumble(RumbleType.kRightRumble, 1);
                }
            } else {
                ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                        ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(false));

                if (driverController != null && codriverController != null) {
                    driverController.setRumble(RumbleType.kLeftRumble, 0);
                    driverController.setRumble(RumbleType.kRightRumble, 0);
                    codriverController.setRumble(RumbleType.kLeftRumble, 0);
                    codriverController.setRumble(RumbleType.kRightRumble, 0);
                }
            }
        } else {
            // If no target, set both speeds to zero
            forwardSpeed = 0.0;
            rotationSpeed = 0.0;

            // Set distance to 0 for shooter math
            double range = 0.0;

            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.DISTANCE_FROM_GOAL, new ShuffleBoardData<Double>(range));
                    
            ShooterUtility.setValuesToShuffleboard(range);
        }

        // Clamp to joystick values
        forwardSpeed = MathUtil.clamp(forwardSpeed, -DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH,
                DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -DriveSubsystem.MAX_ANGULAR_SPEED,
                DriveSubsystem.MAX_ANGULAR_SPEED);

        driveSubsystem.drive(forwardSpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setVoltages(0, 0);

        hubCamera.setLED(VisionLEDMode.kOff);

        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.AIMED, new ShuffleBoardData<Boolean>(false));
    }

    @Override
    public boolean isFinished() {
            return Math.abs(smoothingStack.getAverageYaw()) < YAW_OFFSET;
    }
}
