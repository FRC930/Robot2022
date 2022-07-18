//-------- IMPORTS --------\\

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
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

    // -------- CONSTANTS --------\\

    private final double JOYSTICK_DEADBAND = 0.15;

    // -------- VARIABLES --------\\

    private DriveSubsystem driveSubsystem;
    private XboxController driverController;

    private DoubleSupplier driveStick;
    private DoubleSupplier rotationStick;

    // -------- CONSTRUCTOR --------\\
    /**
     * Initializes a new {@link frc.robot.commands.DriveCommand DriveCommand} with
     * the passed variables
     * 
     * @param dSubsystem  the drive subsystem to control
     * @param dController the driver's controller
     */
    public DriveCommand(
            DriveSubsystem dSubsystem,
            XboxController dController) {
        driveSubsystem = dSubsystem;
        driverController = dController;

        // The left joystick controls the forward power of the drivetrain
        driveStick = () -> -deadbandCube(driverController.getLeftY() * 0.85)
                * DriveSubsystem.DRIVETRAIN_MAX_FREE_SPEED_HIGH;
        // The right joystick controls the rotation of the drivetrain
        rotationStick = () -> -deadbandCube(driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_SPEED * 1.1;

        addRequirements(driveSubsystem);
    }
    // -------- METHODS --------\\

    @Override
    public void execute() {
        double forwardSpeed = driveStick.getAsDouble();
        double rotationSpeed = rotationStick.getAsDouble();

        DifferentialDriveWheelSpeeds wheelSpeeds = driveSubsystem.getWheelSpeeds(forwardSpeed,
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

    /**
     * <h3>deadbandCube</h3>
     * 
     * This method takes in a double and returns the value cubed if it is greater
     * than the deadband, or 0 if it is less than the deadband
     * 
     * A deadband is a small value that is used to prevent the robot from
     * accidentally driving forward or backward
     * 
     * @param stickPos the stick position
     * @return the cubed value of the stick position if it is greater than the
     *         deadband, or 0 if it is less than the deadband
     */
    private double deadbandCube(double stickPos) {
        // Check that the stick value is outside the deadband
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
