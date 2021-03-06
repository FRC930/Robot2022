/**
 * This class simulates a drivetrain for desktop simulation of the robot.
 */

package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("PMD.TooManyFields")

// -------- CLASS --------\\

public class SimulatedDrivetrain {

    // -------- CONSTANTS --------\\

    public static final double kMaxSpeed = 3.0;
    // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI;

    private static final double kTrackWidth = 0.381 * 2;
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = -4096;

    // -------Simulated Hardware-------\\
    private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
    private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
    private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
    private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);

    private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);

    private final Encoder m_leftEncoder = new Encoder(7, 8);
    private final Encoder m_rightEncoder = new Encoder(9, 10);

    private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // Gains are for example purposes only - must be determined for your own
    // robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    // Simulation objects help us simulate our robot
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final Field2d m_fieldSim = new Field2d();
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
            0.3);
    private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
            m_drivetrainSystem, DCMotor.getCIM(2), 8, kTrackWidth, kWheelRadius, null);

    // -------- CONSTRUCTOR --------\\

    public SimulatedDrivetrain() {
        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_rightGroup.setInverted(true);
        SmartDashboard.putData("Field", m_fieldSim);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
        double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

        m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        m_rightGroup.setVoltage(rightOutput + rightFeedforward);
    }

    public void drive(double xSpeed, double rot) {
        setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }

    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    public void resetOdometry(Pose2d pose) {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_drivetrainSimulator.setPose(pose);
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
        m_odometry.resetPosition(pose, pose.getRotation());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public AnalogGyro getGyro(){
        return m_gyro;
    }

    public Encoder getLeftEncoder(){
        return m_leftEncoder;
    }

    public Encoder getRightEncoder(){
        return m_rightEncoder;
    }

    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        m_drivetrainSimulator.setInputs(
                m_leftLeader.get() * RobotController.getInputVoltage(),
                -m_rightLeader.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);

        m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    }

    public void periodic() {
        updateOdometry();
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }
}