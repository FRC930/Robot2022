package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.GyroUtility;
import frc.robot.utilities.ShifterUtility;

/**
 * <h3>DriveSubsystem</h3>
 * 
 * DriveSubsystem represents the drivetrain to our code
 */
public class DriveSubsystem extends SubsystemBase {

    // rpm = 6300
    // with a gear ratio of 6.3, divide 6380 by 6.3 which equals 1012.698
    // divide that value by 60 to get rotations per second : 1012.698 / 60 = 16.678
    // wheel circumference times the rotations per second : 0.1016(wheel dimention)
    // * pi * 16.678 = 5.32

    //----- CONSTANTS -----\\

    // Max linear speed of the robot
    public static final double MAX_SPEED = 5.32; // meters per second
    // Max angular speed of the robot
    public static final double MAX_ANGULAR_SPEED = Math.PI; // 1 rotation per second

    // The ratio between the wheels and the motors in high gear
    public static final double HIGH_GEAR_RATIO = 6.3;
    // The ratio between the wheels and the motors in low gear
    public static final double LOW_GEAR_RATIO = 12.9;

    // The width of the drive base
    public static final double TRACK_WIDTH = 0.381 * 2; // meters // 26.5 inch
    // The radius of the wheels on the robot
    public static final double WHEEL_RADIUS = 0.0555625; // meters // 2 inch
    // The amount of internal encoder units in one motor revolution
    public static final int FALCON_ENCODER_RESOLUTION = 2048; // 2048 CPR
    // The maximum voltage we can send to the motors
    public static final double MAX_VOLTS = 11.0;

    // 6380 is the max free speed (in rpms) of a Falcon 500
    // These are the max speeds of the robot in meters per second
    public static final double DRIVETRAIN_MAX_FREE_SPEED_LOW = 6380.0
            // Convert to rotations per second
            / 60.0
            // Convert to wheel rotations per second
            / LOW_GEAR_RATIO
            // Convert to meters per second
            * (WHEEL_RADIUS * 2 * Math.PI);

    public static final double DRIVETRAIN_MAX_FREE_SPEED_HIGH = 6380.0
            // Convert to rotations per second
            / 60.0
            // Convert to wheel rotations per second
            / HIGH_GEAR_RATIO
            // Convert to meters per second
            * (WHEEL_RADIUS * 2 * Math.PI);

    // Feed Forward Constants
    // KS is static voltage to add to the speed input
    // KV is speed-to-voltage converter basically
    // KA is the acceleration constant
    private final double m_RIGHT_KS = 0.7409;
    private final double m_RIGHT_KV = 2.1937;
    private final double m_RIGHT_KA = 0.17827;
    private final double m_LEFT_KS = 0.73751;
    private final double m_LEFT_KV = 2.1942;
    private final double m_LEFT_KA = 0.18147;
    private final double m_COMBINED_KS = 0.74177;
    private final double m_COMBINED_KV = 2.1936;
    private final double m_COMBINED_KA = 0.34666;

    // PID Constants
    // P is the proportional error gain
    // I is the integral error gain
    // D is the derivative error gain
    private final double m_LEFT_P = 2.464;
    private final double m_LEFT_I = 0.0;
    private final double m_LEFT_D = 0.0;
    private final double m_RIGHT_P = 2.4833;
    private final double m_RIGHT_I = 0.0;
    private final double m_RIGHT_D = 0.0;

    //----- TALONS -----\\

    // Our Falcon 500s
    private final WPI_TalonFX m_leftLeader;
    private final WPI_TalonFX m_leftFollower;
    private final WPI_TalonFX m_rightLeader;
    private final WPI_TalonFX m_rightFollower;

    //----- KINEMATICS -----\\

    // Kinematics to keep track of our wheel speeds
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    //----- FEED FORWARD -----\\

    // The feedforward gains for our motors
    private final SimpleMotorFeedforward m_leftMotorFeedforward = new SimpleMotorFeedforward(m_LEFT_KS, m_LEFT_KV,
            m_LEFT_KA);
    private final SimpleMotorFeedforward m_rightMotorFeedforward = new SimpleMotorFeedforward(m_RIGHT_KS, m_RIGHT_KV,
            m_RIGHT_KA);
    private final SimpleMotorFeedforward m_constraintFeedforward = new SimpleMotorFeedforward(m_COMBINED_KS,
            m_COMBINED_KV, m_COMBINED_KA);

    //----- PID -----\\

    // PID controllers for autonomous
    private final PIDController m_leftPIDController = new PIDController(m_LEFT_P, m_LEFT_I, m_LEFT_D);
    private final PIDController m_rightPIDController = new PIDController(m_RIGHT_P, m_RIGHT_I, m_RIGHT_D);

    //----- ODOMETRY -----\\

    // Set up the voltage constraint for autonomous
    private final DifferentialDriveVoltageConstraint m_voltageConstraint = new DifferentialDriveVoltageConstraint(
            m_constraintFeedforward, m_kinematics, 10);

    // Set up odometry for calculating robot position
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            new Rotation2d(Math.toRadians(GyroUtility.getInstance().getGyro().getYaw())));

    //----- VARIABLES -----\\

    private double m_yawPitchRollValues[] = new double[3];

    //----- CONSTRUCTOR -----\\

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     */
    /**
     * <h3>DriveSubsystem</h3>
     * 
     * Constructs a differential drive object. Sets the encoder distance per pulse
     * and resets the gyro.
     * 
     * @param leftMotorLeaderID    the ID of the left motor leader
     * @param leftMotorFollowerID  the ID of the left motor follower
     * @param rightMotorLeaderID   the ID of the right motor leader
     * @param rightMotorFollowerID the ID of the right motor follower
     */
    public DriveSubsystem(int leftMotorLeaderID, int leftMotorFollowerID, int rightMotorLeaderID,
            int rightMotorFollowerID) {
        m_leftLeader = new WPI_TalonFX(leftMotorLeaderID);
        m_leftFollower = new WPI_TalonFX(leftMotorFollowerID);
        m_rightLeader = new WPI_TalonFX(rightMotorLeaderID);
        m_rightFollower = new WPI_TalonFX(rightMotorFollowerID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_10Ms;

        // Configures all settings on all motors to TalonFX configurations.
        m_leftLeader.configAllSettings(config);
        m_leftFollower.configAllSettings(config);
        m_rightLeader.configAllSettings(config);
        m_rightFollower.configAllSettings(config);

        // Resets motor position.
        m_leftLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_leftFollower.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_rightLeader.getSensorCollection().setIntegratedSensorPosition(0.0, 100);
        m_rightFollower.getSensorCollection().setIntegratedSensorPosition(0.0, 100);

        m_leftLeader.setInverted(InvertType.None);
        // Right side motors are inverted (opposite direction)
        m_rightLeader.setInverted(true);

        refollowDriveMotors();

        // Brake mode (no coasting)
        m_leftLeader.setNeutralMode(NeutralMode.Brake);
        m_leftFollower.setNeutralMode(NeutralMode.Brake);
        m_rightLeader.setNeutralMode(NeutralMode.Brake);
        m_rightFollower.setNeutralMode(NeutralMode.Brake);
    }

    //----- METHODS -----\\

    // Needed to overcome stopMotor() calls by CTRE's WPI motor controls
    // See https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/28
    public void refollowDriveMotors() {
        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);
        // Need to set setInverted to Follow Leader Motors (master)
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
        // Make sure the voltages are not outside of the max range
        leftVoltage = MathUtil.clamp(leftVoltage, -MAX_VOLTS, MAX_VOLTS);
        rightVoltage = MathUtil.clamp(rightVoltage, -MAX_VOLTS, MAX_VOLTS);

        // Send the output to the motors
        m_leftLeader.setVoltage(leftVoltage);
        m_rightLeader.setVoltage(rightVoltage);
    }

    /**
     * <h3>getLeftVoltage</h3>
     * 
     * Gets the voltage of the left motor
     * 
     * @return the left motor output voltage
     */
    public double getLeftVoltage() {
        return m_leftLeader.getMotorOutputVoltage();
    }

    /**
     * <h3>getRightVoltage</h3>
     * 
     * Gets the voltage of the right motor
     * 
     * @return the right motor output voltage
     */
    public double getRightVoltage() {
        return m_rightLeader.getMotorOutputVoltage();
    }

    /**
     * <h3>getKinematics</h3>
     * 
     * Gets the kinematics
     * 
     * @return the kinematics used by the drivetrain
     */
    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Calculate the feedforward with the left controller
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the output of the feedforward controller (in volts)
     */
    public double calculateLeftFeedforward(double velocity) {
        return m_leftMotorFeedforward.calculate(velocity);
    }

    /**
     * Calculate the feedforward with the left controller
     * Overloaded method to allow the input of acceleration
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the output of the feedforward contoller (in volts)
     */
    public double calculateLeftFeedforward(double velocity, double acceleration) {
        return m_leftMotorFeedforward.calculate(velocity, acceleration);
    }

    /**
     * Calculate the feedforward with the right controller
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the output of the feedforward controller (in volts)
     */
    public double calculateRightFeedforward(double velocity) {
        return m_rightMotorFeedforward.calculate(velocity);
    }

    /**
     * Calculate the feedforward with the right controller
     * Overloaded method to allow the input of acceleration
     * 
     * @param velocity the velocity for which to calculate feedforward
     * @return the output of the feedforward controller (in volts)
     */
    public double calculateRightFeedforward(double velocity, double acceleration) {
        return m_rightMotorFeedforward.calculate(velocity, acceleration);
    }

    /**
     * Calculate the left PID values
     * 
     * @param speed  the current speed
     * @param target the target speed
     * @return the PID corrected speed
     */
    public double calculateLeftPID(double speed, double target) {
        return m_leftPIDController.calculate(speed, target);
    }

    /**
     * Calculate the left PID values
     * 
     * @param speed  the current speed
     * @param target the target speed
     * @return the PID corrected speed
     */
    public double calculateRightPID(double speed, double target) {
        return m_rightPIDController.calculate(speed, target);
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
        return speed / DRIVETRAIN_MAX_FREE_SPEED_HIGH * MAX_VOLTS;
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Convert speeds to volts
        double leftOutput = speeds.leftMetersPerSecond * MAX_VOLTS /
                DriveSubsystem.MAX_SPEED;
        double rightOutput = speeds.rightMetersPerSecond * MAX_VOLTS /
                DriveSubsystem.MAX_SPEED;

        setVoltages(leftOutput, rightOutput);
    }

    /**
     * Gets the reading from the left encoder
     * 
     * @return the encoder speed
     */
    public double getLeftEncoder() {
        return m_leftLeader.getSelectedSensorVelocity()
                // Multiply by 10 to get encoder units per second
                * 10
                // Divide by the number of ticks in a rotation
                / FALCON_ENCODER_RESOLUTION
                // Multiply by the circumference of the wheel
                * (2 * Math.PI * WHEEL_RADIUS) / (ShifterUtility.getShifterState() ? LOW_GEAR_RATIO : HIGH_GEAR_RATIO);
    }

    /**
     * Gets the reading from the right encoder
     * 
     * @return the encoder speed
     */
    public double getRightEncoder() {
        return m_rightLeader.getSelectedSensorVelocity()
                // Multiply by 10 to get encoder units per second
                * 10
                // Divide by the number of ticks in a rotation
                / FALCON_ENCODER_RESOLUTION
                // Multiply by the circumference of the wheel
                * (2 * Math.PI * WHEEL_RADIUS) / (ShifterUtility.getShifterState() ? LOW_GEAR_RATIO : HIGH_GEAR_RATIO);
    }

    /**
     * <h3>getRawLeftSensorPosition</h3>
     * 
     * This method gets the position of the right encoder
     * 
     * @return the position of the right encoder in raw encoder units
     * @see {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX#getSelectedSensorPosition()
     *      getSelectedSensorPosition()}
     */
    public double getRawLeftSensorPosition() {
        return m_leftLeader.getSelectedSensorPosition();
    }

    /**
     * <h3>getRawRightSensorPosition</h3>
     * 
     * This method gets the position of the right encoder
     * 
     * @return the position of the left encoder in raw encoder units
     * @see {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX#getSelectedSensorPosition()
     *      getSelectedSensorPosition()}
     */
    public double getRawRightSensorPosition() {
        return m_rightLeader.getSelectedSensorPosition();
    }

    /**
     * <h3>resetLeftPID</h3>
     * 
     * This method resets the left PID controller (only useful when using integral
     * and derivative)
     */
    public void resetLeftPID() {
        m_leftPIDController.reset();
    }

    /**
     * <h3>resetRightPID</h3>
     * 
     * This method resets the right PID controller (only useful when using integral
     * and derivative)
     */
    public void resetRightPID() {
        m_rightPIDController.reset();
    }

    /**
     * <h3>resetEncoders</h3>
     * 
     * Resets the encoders to zero. This will affect the selected sensor position,
     * but not the absolute position
     */
    public void resetEncoders() {
        m_leftLeader.setSelectedSensorPosition(0.0);
        m_rightLeader.setSelectedSensorPosition(0.0);
    }

    /**
     * <h3>getVoltageConstraint</h3>
     * 
     * Get the voltage constraint used for autonomous
     * 
     * @return the autonomous voltage constraint
     */
    public DifferentialDriveVoltageConstraint getVoltageContraint() {
        return m_voltageConstraint;
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
        GyroUtility.getInstance().getGyro().getYawPitchRoll(m_yawPitchRollValues);
        return Math.IEEEremainder(m_yawPitchRollValues[0], 360);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @return m_odometry .
     */
    public DifferentialDriveOdometry getOdometry() {
        return m_odometry;
    }

    /**
     * <h3>periodic</h3>
     * 
     * Called every loop iteration when the scheduler runs.
     * <p>
     * This will update the odometry for automous
     * 
     * @see super {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic
     *      periodic} method
     */
    @Override
    public void periodic() {
        // Update odometry using the gyro and the wheel rotations
        m_odometry.update(
                // Create a new Rotation2d object with the reading from the pigeon
                new Rotation2d(Math.toRadians(GyroUtility.getInstance().getGyro().getYaw())),
                // Convert raw sensor units to meters
                getRawLeftSensorPosition()
                        // Convert raw sensor units to wheel rotations
                        * (1.0 / 2048.0)
                        // Convert wheel rotations to meters
                        * (WHEEL_RADIUS * Math.PI * 2)
                        // Divide by the current gear ratio
                        / HIGH_GEAR_RATIO,
                // Convert raw sensor units to meters
                getRawRightSensorPosition()
                        // Convert raw sensor units to wheel rotations
                        * (1.0 / 2048.0)
                        // Convert wheel rotations to meters
                        * (WHEEL_RADIUS * Math.PI * 2)
                        // Divide by the current gear ratio
                        / HIGH_GEAR_RATIO);
    }
}
