package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * <h3>DriveCommand</h3>
 * 
 * <p>
 * Drives the robot with passed joysticks
 * </p>
 */
public class DriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private XboxController driverController;

    private final double JOYSTICK_DEADBAND = 0.15;

    private DoubleSupplier driveStick;
    private DoubleSupplier rotationStick;

    /**
     * Initializes a new {@link frc.robot.commands.DriveCommand DriveCommand} with
     * the passed variables
     * 
     * @param dSubsystem  the drive subsystem
     * @param dController the driver controller
     */
    public DriveCommand(DriveSubsystem dSubsystem, XboxController dController) {
        driveSubsystem = dSubsystem;
        driverController = dController;

        driveStick = () -> -deadbandCube(driverController.getLeftY()) * DriveSubsystem.kMaxSpeed;
        rotationStick = () -> -deadbandCube(driverController.getRightX()) * DriveSubsystem.kMaxAngularSpeed;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // driveSubsystem.drive(driveStick.getAsDouble(), rotationStick.getAsDouble());
        // driveSubsystem.getLeftEncoder();
        // driveSubsystem.getRightEncoder();
        DifferentialDriveWheelSpeeds wheelSpeeds = driveSubsystem.getWheelSpeeds(driveStick.getAsDouble(),
                rotationStick.getAsDouble());
        driveSubsystem.setVoltages(driveSubsystem.speedToVoltage(
                driveSubsystem.calculateLeftFeedforward(wheelSpeeds.leftMetersPerSecond)),
                driveSubsystem
                        .speedToVoltage(driveSubsystem.calculateRightFeedforward(wheelSpeeds.rightMetersPerSecond)));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0);
    }

    private double deadbandCube(double stickPos) {
        if (stickPos < JOYSTICK_DEADBAND && stickPos > -JOYSTICK_DEADBAND) {
            return 0;
        } else {
            return Math.pow(stickPos, 3);
        }
    }
}
