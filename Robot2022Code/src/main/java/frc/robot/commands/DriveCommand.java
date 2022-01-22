package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;

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
    private PigeonIMU m_gyro;

    private final double JOYSTICK_DEADBAND = 0.15;

    private DoubleSupplier driveStick;
    private DoubleSupplier rotationStick;

    private DifferentialDriveOdometry m_odometry;

    /**
     * Initializes a new {@link frc.robot.commands.DriveCommand DriveCommand} with
     * the passed variables
     * 
     * @param dSubsystem  the drive subsystem
     * @param dController the driver controller
     */
    public DriveCommand(DriveSubsystem dSubsystem, EndgameMotorSubsystem eSubsystem, XboxController dController) {
        driveSubsystem = dSubsystem;
        driverController = dController;

        m_gyro = new PigeonIMU(eSubsystem.getEndgameMotorSlave());

        driveStick = () -> -deadbandCube(driverController.getLeftY()) * DriveSubsystem.kMaxSpeed;
        rotationStick = () -> -deadbandCube(driverController.getRightX()) * DriveSubsystem.kMaxAngularSpeed;

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(m_gyro.getFusedHeading())));

        // We are not adding endgame motor subsystem as a requirement because we are not
        // using the subsystem in the command at all
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        DifferentialDriveWheelSpeeds wheelSpeeds = driveSubsystem.getWheelSpeeds(driveStick.getAsDouble(),
                rotationStick.getAsDouble());
        driveSubsystem.setVoltages(driveSubsystem.speedToVoltage(
                driveSubsystem.calculateLeftFeedforward(wheelSpeeds.leftMetersPerSecond)),
                driveSubsystem
                        .speedToVoltage(driveSubsystem.calculateRightFeedforward(wheelSpeeds.rightMetersPerSecond)));
        m_odometry.update(new Rotation2d(Math.toRadians(m_gyro.getFusedHeading())),
                driveSubsystem.getRawLeftSensorPosition() * ((1.0 / 2048.0) * DriveSubsystem.kWheelRadius * Math.PI)
                        / DriveSubsystem.highGearRatio,
                driveSubsystem.getRawRightSensorPosition() * ((1.0 / 2048.0) * DriveSubsystem.kWheelRadius * Math.PI)
                        / DriveSubsystem.highGearRatio);
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
