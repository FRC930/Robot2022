//-------- IMPORTS --------\\

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
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

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

    //-------- CONSTANTS --------\\

    private final double JOYSTICK_DEADBAND = 0.15;

    //-------- VARIABLES --------\\

    private DriveSubsystem driveSubsystem;
    private XboxController driverController;

    private DoubleSupplier driveStick;
    private DoubleSupplier rotationStick;

    private PIDController turnController = new PIDController(0.000001, 0, 0);

    //-------- CONSTRUCTOR --------\\
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
            XboxController dController) {
        driveSubsystem = dSubsystem;
        driverController = dController;

        driveStick = () -> -deadbandCube(driverController.getLeftY() * 0.85) * DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH;
                // * (ShifterUtility.getShifterState() ? DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_LOW
                //         : DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH);
        rotationStick = () -> -deadbandCube(driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_SPEED;

        // We are not adding endgame motor subsystem as a requirement because we are not
        // using the subsystem in the command at all
        addRequirements(driveSubsystem);
    }
    //-------- METHODS --------\\

    @Override
    public void execute() {
        // No need to modify drive speed axis
        double xStick = driveStick.getAsDouble();
        double rotationSpeed = rotationStick.getAsDouble();

        DifferentialDriveWheelSpeeds wheelSpeeds = driveSubsystem.getWheelSpeeds(xStick,
                rotationSpeed);

        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.testingTab, ShuffleboardKeys.LEFT_SPEED,
                new ShuffleBoardData<Double>(wheelSpeeds.leftMetersPerSecond));
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.testingTab,
                ShuffleboardKeys.RIGHT_SPEED,
                new ShuffleBoardData<Double>(wheelSpeeds.rightMetersPerSecond));

        driveSubsystem.setVoltages(
                // Calculate feedforward with the feedforward controller in drive subsystem
                driveSubsystem.calculateLeftFeedforward(
                        // Use the speed to voltage method in the drive subsystem
                        wheelSpeeds.leftMetersPerSecond),
                // Same deal here, feedforward using the helper method
                driveSubsystem.calculateRightFeedforward(
                        // Again, speed to voltage
                        wheelSpeeds.rightMetersPerSecond));
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
