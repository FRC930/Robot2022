package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.GyroUtility;
import frc.robot.utilities.ShifterUtility;
import frc.robot.utilities.SimulatedDrivetrain;

/**
 * <h3>DriveSubsystem</h3>
 * 
 * DriveSubsystem represents the drivetrain to our code
 * 
 * @author Alexander Taylor
 * @since 22 January 2022
 * @version 1.0
 */
public class DriveSubsystem extends SubsystemBase {

    public static final double highGearRatio = 6.3;
    public static final double lowGearRatio = 12.9;

    public static final double kTrackWidth = 0.381 * 2; // meters // 26.5 inch
    public static final double kWheelRadius = Units.inchesToMeters(3.875 / 2); // meters // 3 7/8 inch
    public static final int kEncoderResolution = 2048; // 2048 CPR
    public static final double kMaxVolts = 12.0;

    // 6380 is the max free speed (in rpms) of a Falcon 500
    public static final double DRIVETRAIN_MAX_FREE_SPEED_LOW = 6380.0 / 60.0 / lowGearRatio
            * (kWheelRadius * 2 * Math.PI);
    public static final double DRIVETRAIN_MAX_FREE_SPEED_HIGH = 6380.0 / 60.0 / highGearRatio
            * (kWheelRadius * 2 * Math.PI);

    // rpm = 6300
    // with a gear ratio of 6.3, divide 6380 by 6.3 which equals 1012.698
    // divide that value by 60 to get rotations per second : 1012.698 / 60 = 16.678
    // wheel circumference times the rotations per second : 0.098425(wheel dimension) * pi * 16.678 = 5.157
    public static final double kMaxSpeed = DRIVETRAIN_MAX_FREE_SPEED_HIGH; // meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1 rotation per second

    private final WPI_TalonFX m_leftLeader;
    private final WPI_TalonFX m_leftFollower;
    private final WPI_TalonFX m_rightLeader;
    private final WPI_TalonFX m_rightFollower;

    // Feed Forward Constants
    private final double m_rightKS = 0.7409;
    private final double m_rightKV = 2.1937;
    private final double m_rightKA = 0.17827;
    private final double m_leftKS = 0.73751;
    private final double m_leftKV = 2.1942;
    private final double m_leftKA = 0.18147;
    private final double m_combinedKS = 0.74177;
    private final double m_combinedKV = 2.1936;
    private final double m_combinedKA = 0.34666;

    // PID Constants
    // Getting I values
    // 1 divided by loop period in minutes times 2
    // our loop period is 20 miliseconds which calculated into minutes is : 20(Loop
    // period) / 60000(Miliseconds in one minute) = 0.000333
    // calculated estimated I = 1/(0.000333 * 2)
    private final double m_loopPeriodPerMin = 0.000333;
    private final double m_leftP = 2.464;
    private final double m_leftI = 0.0;// 1/(m_loopPeriodPerMin * 2);
    private final double m_leftD = 0.0;
    private final double m_rightP = 2.4833;
    private final double m_rightI = 0.0;// 1/(m_loopPeriodPerMin * 2);
    private final double m_rightD = 0.0;
    // MotorControllerGroup leftGroup;
    // MotorControllerGroup rightGroup;
    /*
     * private final SpeedControllerGroup m_leftGroup = new
     * SpeedControllerGroup(m_leftLeader, m_leftFollower); private final
     * SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader,
     * m_rightFollower);
     */

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward leftMotorFeedforward = new SimpleMotorFeedforward(m_leftKS, m_leftKV,
            m_leftKA);
    private final SimpleMotorFeedforward rightMotorFeedforward = new SimpleMotorFeedforward(m_rightKS, m_rightKV,
            m_rightKA);
    private final SimpleMotorFeedforward constraintFeedforward = new SimpleMotorFeedforward(m_combinedKS,
            m_combinedKV, m_combinedKA);

    private final PIDController leftPIDController = new PIDController(m_leftP, m_leftI, m_leftD);
    private final PIDController rightPIDController = new PIDController(m_rightP, m_rightI, m_rightD);

    private boolean shifterState;

    private final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
            constraintFeedforward, getKinematics(), 10);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(GyroUtility.getInstance().getGyro().getFusedHeading())));

    private double yawPitchRollValues[] = new double[3];

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     */
    public DriveSubsystem(int leftMotorLeaderID, int leftMotorFollowerID, int rightMotorLeaderID,
            int rightMotorFollowerID) {
        m_leftLeader = new WPI_TalonFX(leftMotorLeaderID);
        m_leftFollower = new WPI_TalonFX(leftMotorFollowerID);
        m_rightLeader = new WPI_TalonFX(rightMotorLeaderID);
        m_rightFollower = new WPI_TalonFX(rightMotorFollowerID);

        // leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
        // rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);

        shifterState = false;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;

        m_leftLeader.configAllSettings(config);
        m_leftFollower.configAllSettings(config);
        m_rightLeader.configAllSettings(config);
        m_rightFollower.configAllSettings(config);

        m_leftLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_leftFollower.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_rightLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_rightFollower.getSensorCollection().setIntegratedSensorPosition(0.0, 100);

        m_leftLeader.setInverted(InvertType.None);
        // Right side motors are inverted (opposite direction)
        m_rightLeader.setInverted(true);

        refollowDriveMotors();

        m_leftLeader.setNeutralMode(NeutralMode.Brake);
        m_leftFollower.setNeutralMode(NeutralMode.Brake);
        m_rightLeader.setNeutralMode(NeutralMode.Brake);
        m_rightFollower.setNeutralMode(NeutralMode.Brake);
    }

    // Needed to overcome stopMotor() calls by CTRE's WPI motor controls
    // See https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/28
    public void refollowDriveMotors() {
        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);
        // Need to set setInverted to Follow Laader Motors (master)
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_rightFollower.setInverted(InvertType.FollowMaster);
    }

    /**
     * <h3>setMototBreakMode</h3>
     * 
     * Sets left and right motors to a certain breakmode: auto is brake; teleop is
     * coast
     * 
     * @param brakeMode
     */
    public void setMotorBrakeMode(NeutralMode brakeMode) {
        m_leftLeader.setNeutralMode(brakeMode);
        m_leftFollower.setNeutralMode(brakeMode);
        m_rightLeader.setNeutralMode(brakeMode);
        m_rightFollower.setNeutralMode(brakeMode);
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
     * <h3>getLeftVoltage</h3>
     * 
     * Gets the voltage of the left motor
     * 
     */
    public double getLeftVoltage() {
        return m_leftLeader.getMotorOutputVoltage();
    }

    /**
     * <h3>getRightVoltage</h3>
     * 
     * Gets the voltage of the right motor
     * 
     */
    public double getRightVoltage() {
        return m_rightLeader.getMotorOutputVoltage();
    }

    /**
     * <h3>getKinematics</h3>
     * 
     * Gets the kinematics
     * 
     */
    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
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
     * Calculate the feedforward with the left controller
     * Overloaded method to allow the input of acceleration
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the feedforward modified
     */
    public double calculateLeftFeedforward(double velocity, double acceleration) {
        return leftMotorFeedforward.calculate(velocity, acceleration);
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
     * Calculate the feedforward with the right controller
     * Overloaded method to allow the input of acceleration
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the feedforward modified
     */
    public double calculateRightFeedforward(double velocity, double acceleration) {
        return rightMotorFeedforward.calculate(velocity, acceleration);
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
     * Returns the speeds in the differential drive
     * Overloaded getWheelSpeed that just gets the motor encoder position
     * 
     * @return A DifferentialDriveWheelSpeeds object, has the rotations of the left
     *         and right wheels
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoder(), getRightEncoder());
    }

    /**
     * Convert speed to voltage
     * 
     * @param speed the speed at which the motor should turn
     * @return the volatage at which to run the motor
     */
    public double speedToVoltage(double speed) {
        return speed / DRIVETRAIN_MAX_FREE_SPEED_HIGH * kMaxVolts;
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Convert speeds to volts
        double leftOutput = speeds.leftMetersPerSecond * kMaxVolts /
                DriveSubsystem.kMaxSpeed;
        double rightOutput = speeds.rightMetersPerSecond * kMaxVolts /
                DriveSubsystem.kMaxSpeed;

        setVoltages(leftOutput, rightOutput);
    }

    /**
     * Gets the reading from the left encoder
     * 
     * @return the encoder speed
     */
    public double getLeftEncoder() {
        return
        // m_leftLeader.getSensorCollection().getIntegratedSensorVelocity()
        m_leftLeader.getSelectedSensorVelocity()
                // Multiply by 10 to get encoder units per second
                * 10
                // Divide by the number of ticks in a rotation
                / kEncoderResolution
                // Multiply by the circumference of the wheel
                * (2 * Math.PI * kWheelRadius) / (ShifterUtility.getShifterState() ? lowGearRatio : highGearRatio);
    }

    /**
     * Gets the reading from the right encoder
     * 
     * @return the encoder speed
     */
    public double getRightEncoder() {
        // Multiply by 10 to get encoder units per second
        return m_rightLeader.getSelectedSensorVelocity()
                // m_rightLeader.getSensorCollection().getIntegratedSensorVelocity()
                * 10 / kEncoderResolution
                * (2 * Math.PI * kWheelRadius) / (ShifterUtility.getShifterState() ? lowGearRatio : highGearRatio);
    }

    /**
     * <h3>getRawLeftSensorPosition</h3>
     * 
     * This method gets the position of the right encoder
     * 
     * @return the position of the right encoder in raw encoder units
     * @see {@link com.ctre.phoenix.motorcontrol.TalonFXSensorCollection#getIntegratedSensorPosition()
     *      getIntegratedSensorPosition()}
     */
    public double getRawLeftSensorPosition() {
        // return m_leftLeader.getSensorCollection().getIntegratedSensorPosition();
        return m_leftLeader.getSelectedSensorPosition();
    }

    /**
     * <h3>getRawRightSensorPosition</h3>
     * 
     * This method gets the position of the right encoder
     * 
     * @return the position of the left encoder in raw encoder units
     * @see {@link com.ctre.phoenix.motorcontrol.TalonFXSensorCollection#getIntegratedSensorPosition()
     *      getIntegratedSensorPosition()}
     */
    public double getRawRightSensorPosition() {
        // return m_rightLeader.getSensorCollection().getIntegratedSensorPosition();
        return m_rightLeader.getSelectedSensorPosition();
    }

    /**
     * <h3>resetLeftPID</h3>
     * 
     * This method resets the left PID controller
     * 
     */
    public void resetLeftPID() {
        leftPIDController.reset();
    }

    /**
     * <h3>resetRightPID</h3>
     * 
     * This method resets the right PID controller
     * 
     */
    public void resetRightPID() {
        rightPIDController.reset();
    }

    public void resetEncoders() {
        m_leftLeader.setSelectedSensorPosition(0.0);
        m_rightLeader.setSelectedSensorPosition(0.0);
    }

    // TODO: Look into making this into a separate state machine class
    /**
     * Set the shifting piston state
     * 
     * @param state the state to set
     */
    public void setPistonState(boolean state) {
        shifterState = state;
    }

    /**
     * Gets the state of the shifter
     * 
     * @return the state of the shifter
     */
    public boolean getPistonState() {
        return shifterState;
    }

    public DifferentialDriveVoltageConstraint getVoltageContraint() {
        return voltageConstraint;
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

    /**
     * Returns the heading of the gyro.
     *
     * @return the heading of the gyro.
     */
    public double getHeading() {
        GyroUtility.getInstance().getGyro().getYawPitchRoll(yawPitchRollValues);
        return Math.IEEEremainder(yawPitchRollValues[0], 360);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @return m_odometry .
     */
    public DifferentialDriveOdometry getOdometry() {
        return m_odometry;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("left encoder", getRawLeftSensorPosition());
        // SmartDashboard.putNumber("right encoder", getRawRightSensorPosition());
        m_odometry.update(
                // Create a new Rotation2d object with the reading from the pigeon
                new Rotation2d(Math.toRadians(GyroUtility.getInstance().getGyro().getFusedHeading())),
                // Convert raw sensor units to meters
                // TODO: Check if we need to multiply by 2*pi*r because circumference
                getRawLeftSensorPosition() *
                        ((1.0 / 2048.0) * kWheelRadius * Math.PI * 2)
                        // Divide by the current gear ratio because the motors turn more than the wheels
                        / highGearRatio,
                // Convert raw sensor units to meters
                // TODO: Check if we need to multiply by 2*pi*r because circumference
                getRawRightSensorPosition() * ((1.0 / 2048.0) * kWheelRadius * Math.PI * 2)
                // Divide by the current gear ratio because the motors turn more than the wheels
                        / highGearRatio);
        Pose2d robotPosition = m_odometry.getPoseMeters();
        // SmartDashboard.putNumber("robotPositionX", robotPosition.getX());
        // SmartDashboard.putNumber("robotPositionY", robotPosition.getY());
    }
}
