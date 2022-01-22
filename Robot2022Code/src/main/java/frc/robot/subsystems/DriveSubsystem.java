package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public static final double kMaxSpeed = 3; // meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final double highGearRatio = 6.3;
    public static final double lowGearRatio = 12.9;

    public static final double kTrackWidth = 0.381 * 2; // meters // 26.5 inch
    public static final double kWheelRadius = 0.0508; // meters // 2 inch
    public static final int kEncoderResolution = 2048; // 2048 CPR
    public static final double kMaxVolts = 12.0;

    private final WPI_TalonFX m_leftLeader;
    // private final SpeedController m_leftFollower = new WPI_TalonFX(2);
    private final WPI_TalonFX m_rightLeader;
    // private final SpeedController m_rightFollower = new WPI_TalonFX(4);

    /*
     * private final SpeedControllerGroup m_leftGroup = new
     * SpeedControllerGroup(m_leftLeader, m_leftFollower); private final
     * SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader,
     * m_rightFollower);
     */

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Gains are for example purposes only - must be determined for your own robot!
    public final SimpleMotorFeedforward leftMotorFeedforward = new SimpleMotorFeedforward(0.61037, 0.68157, 0.023755);
    public final SimpleMotorFeedforward rightMotorFeedforward = new SimpleMotorFeedforward(0.62728, 0.68254, 0.021885);

    private final PIDController leftPIDController = new PIDController(0.50405, 0, 0);
    private final PIDController rightPIDController = new PIDController(0.47029, 0, 0);

    private boolean shifterState;

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     */
    public DriveSubsystem(int leftMotorID, int rightMotorID) {
        m_leftLeader = new WPI_TalonFX(leftMotorID);
        m_rightLeader = new WPI_TalonFX(rightMotorID);

        shifterState = false;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;

        m_leftLeader.configAllSettings(config);
        m_rightLeader.configAllSettings(config);

        m_leftLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_rightLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);

        m_leftLeader.setInverted(true);
    }

    /**
     * <h3>setVoltages</h3>
     * 
     * Sets the voltages of the drivetrain to the passed values
     * 
     * @param leftVoltage  the left voltage
     * @param rightVoltage the right voltage
     */
    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftVoltage = MathUtil.clamp(leftVoltage, -11.0, 11.0);
        rightVoltage = MathUtil.clamp(rightVoltage, -11.0, 11.0);

        m_leftLeader.setVoltage(leftVoltage);
        m_rightLeader.setVoltage(rightVoltage);
    }

    /**
     * Calculate the feedforward with the left controller
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the feedforward modified velocity
     */
    public double calculateLeftFeedforward(double velocity) {
        return leftMotorFeedforward.calculate(velocity);
    }

    /**
     * Calculate the feedforward with the right controller
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the feedforward modified velocity
     */
    public double calculateRightFeedforward(double velocity) {
        return rightMotorFeedforward.calculate(velocity);
    }

    /**
     * Calculate the left PID values
     * 
     * @param speed  the current speed
     * @param target the target speed
     * @return the PID corrected speed
     */
    public double calculateLeftPID(double speed, double target) {
        return leftPIDController.calculate(speed, target);
    }

    /**
     * Calculate the left PID values
     * 
     * @param speed  the current speed
     * @param target the target speed
     * @return the PID corrected speed
     */
    public double calculateRightPID(double speed, double target) {
        return rightPIDController.calculate(speed, target);
    }

    /**
     * Convert stick input values to wheel speeds
     * 
     * @param xSpeed speed stick
     * @param rot    rotation stick
     * @return the wheel speeds from the sticks
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(double xSpeed, double rot) {
        return m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    }

    /**
     * Convert speed to voltage
     * 
     * @param speed the speed at which the motor should turn
     * @return the volatage at which to run the motor
     */
    public double speedToVoltage(double speed) {
        return speed * kMaxVolts;
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        double leftOutput = speeds.leftMetersPerSecond * kMaxVolts /
                DriveSubsystem.kMaxSpeed;
        double rightOutput = speeds.rightMetersPerSecond * kMaxVolts /
                DriveSubsystem.kMaxSpeed;

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
    public double getLeftEncoder() {
        // Multiply by 10 to get encoder units per second
        return m_leftLeader.getSensorCollection().getIntegratedSensorVelocity() * 10 / kEncoderResolution
                * (2 * Math.PI * kWheelRadius);
    }

    /**
     * Gets the reading from the right encoder
     * 
     * @return the encoder speed
     */
    public double getRightEncoder() {
        // Multiply by 10 to get encoder units per second
        return m_rightLeader.getSensorCollection().getIntegratedSensorVelocity() * 10 / kEncoderResolution
                * (2 * Math.PI * kWheelRadius);
    }

    public double getRawLeftSensorPosition() {
        return m_leftLeader.getSelectedSensorPosition();
    }

    public double getRawRightSensorPosition() {
        return m_rightLeader.getSelectedSensorPosition();
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
}
