package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;

public class DriveSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 1; // meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private static final double highGearRatio = 6.3;
    private static final double lowGearRatio = 12.9;

    private static final double kTrackWidth = 0.381 * 2; // meters // 26.5 inch
    private static final double kWheelRadius = 0.0508; // meters // 2 inch
    private static final int kEncoderResolution = 2048; // 2048 CPR
    private static final double kMaxVolts = 12.0;

    private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(1);
    // private final SpeedController m_leftFollower = new WPI_TalonFX(2);
    private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(2);
    // private final SpeedController m_rightFollower = new WPI_TalonFX(4);

    /*
     * private final SpeedControllerGroup m_leftGroup = new
     * SpeedControllerGroup(m_leftLeader, m_leftFollower); private final
     * SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader,
     * m_rightFollower);
     */

    private final PigeonIMU m_gyro = new PigeonIMU(new TalonSRX(4));

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    private final DifferentialDriveOdometry m_odometry;

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.61961, 2.01492, 0.13214);// (0.66764,
                                                                                                               // 0.10838,
                                                                                                               // 0.0056832);

    private boolean shifterState;

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     */
    public DriveSubsystem() {
        shifterState = false;

        m_gyro.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);

        SmartDashboard.putNumber("Left kP", 1);
        SmartDashboard.putNumber("Right kP", 1);

        m_leftLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_rightLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);

        m_leftLeader.setInverted(true);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(m_gyro.getFusedHeading())));
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        SmartDashboard.putNumber("Left wheel speed", speeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right wheel speed", speeds.rightMetersPerSecond);

        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        double leftPIDOutput = m_leftPIDController.calculate(getLeftEncoder(),
                speeds.leftMetersPerSecond);
        double leftOutput = leftPIDOutput * kMaxVolts /
                DriveSubsystem.kMaxSpeed;

        double rightPIDOutput = m_rightPIDController.calculate(getRightEncoder(),
                speeds.rightMetersPerSecond);
        double rightOutput = rightPIDOutput * kMaxVolts /
                DriveSubsystem.kMaxSpeed;

        leftOutput += leftFeedforward;
        rightOutput += rightFeedforward;

        SmartDashboard.putNumber("Left feedforward", leftFeedforward);
        SmartDashboard.putNumber("Right feedforward", rightFeedforward);

        leftOutput = MathUtil.clamp(leftOutput, -11.0, 11.0);
        rightOutput = MathUtil.clamp(rightOutput, -11.0, 11.0);
        m_leftLeader.setVoltage(leftOutput);
        m_rightLeader.setVoltage(rightOutput);
    }

    /**
     * Gets the reading from the left encoder
     * 
     * @return the encoder speed
     */
    private double getLeftEncoder() {
        return m_leftLeader.getSensorCollection().getIntegratedSensorVelocity() / 10 / kEncoderResolution * 0.3192;
    }

    /**
     * Gets the reading from the right encoder
     * 
     * @return the encoder speed
     */
    private double getRightEncoder() {
        return m_rightLeader.getSensorCollection().getIntegratedSensorVelocity() / 10 / kEncoderResolution * 0.3192;
    }

    /**
     * Set the shifting piston state
     * 
     * @param state the state to set
     */
    public void setPistonState(boolean state) {
        shifterState = state;
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot) {
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
    }

    /** Updates the field-relative position. */
    public void updateOdometry() {

        // TODO: Attach a pigeon IMU and figure out how this whole distance thing works
        // we'll need a switch case to differentiate between high and low gear

        m_odometry.update(new Rotation2d(Math.toRadians(m_gyro.getFusedHeading())),
                m_leftLeader.getSelectedSensorPosition() * ((1.0 / 2048.0) * kWheelRadius * Math.PI) / highGearRatio,
                m_rightLeader.getSelectedSensorPosition() * ((1.0 / 2048.0) * kWheelRadius * Math.PI) / highGearRatio);
        m_odometry.update(new Rotation2d(Math.toRadians(m_gyro.getFusedHeading())),
                m_leftLeader.getSelectedSensorPosition() * ((1.0 / 2048.0) * kWheelRadius * Math.PI) / highGearRatio,
                m_rightLeader.getSelectedSensorPosition() * ((1.0 / 2048.0) * kWheelRadius * Math.PI) / highGearRatio);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pigeon fused heading", m_gyro.getFusedHeading() % 360);
        SmartDashboard.putNumber("Left encoder value", getLeftEncoder());
        SmartDashboard.putNumber("Right encoder value", getRightEncoder());
        m_leftPIDController.setP(SmartDashboard.getNumber("Left kP", 1));
        m_rightPIDController.setP(SmartDashboard.getNumber("Right kP", 1));
    }
}
