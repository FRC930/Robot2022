//----- IMPORTS -----\\

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerMotorCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LEDCommand.LEDPatterns;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.commands.autocommands.AutoCommandManager;
import frc.robot.commands.autocommands.AutoCommandManager.subNames;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.endgamecommands.EndgameArmCommand;
import frc.robot.commands.endgamecommands.EndgameArmRevCommand;
import frc.robot.commands.endgamecommands.EndgameCloseClawCommand;
import frc.robot.commands.endgamecommands.EndgameManagerCommand;
import frc.robot.commands.endgamecommands.EndgameRotateArmCommand;
import frc.robot.commands.endgamecommands.EndgameRotateArmCommand.EndgamePosition;
import frc.robot.commands.intakecommands.intakePistonCommands.DisengageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.StopIntakeMotorsCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import frc.robot.commands.shootercommands.AdjustHoodCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.SimulatedDrivetrain;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.DriveCameraUtility.BallColor;
import frc.robot.utilities.EndgameSensorUtility;
import frc.robot.utilities.PhotonVisionUtility;
import frc.robot.utilities.ShooterUtility;

//----- CLASS -----\\
/**
 * <h3>RobotContainer</h3>
 * 
 * Manages all robot hardware and software.
 */
public class RobotContainer {

    //----- XBOX CONTROLLER(S) -----\\

    // Driver Controller
    private final ControllerManager m_driverController = new ControllerManager(0);
    // Codriver Controller
    private final ControllerManager m_codriverController = new ControllerManager(1);

    //----- CAMERA CONSTANTS -----\\

    public static final int CAMERA_WIDTH = 160;
    public static final int CAMERA_HEIGHT = 120;
    public static final int CAMERA_FPS = 30;

    //----- INTAKE -----\\

    // Intake Motor Subsystem
    private final IntakeMotorSubsystem m_intakeMotorSubsystem;
    // Intake Piston Subsystem
    private final IntakePistonSubsystem m_intakePistonSubsystem;

    // Intake Motor Commands
    private final RunIntakeMotorsCommand m_runIntakeMotorsCommand;
    private final RunIntakeMotorsCommand m_reverseIntakeMotorsCommand;
    private final StopIntakeMotorsCommand m_stopIntakeMotorsCommand;

    // Intake Piston Commands
    private final EngageIntakePistonsCommand m_engageIntakePistonsCommand;
    private final DisengageIntakePistonsCommand m_disengageIntakePistonsCommand;

    //----- DRIVETRAIN MOTORS -----\\

    // Drive Subsystem
    private final DriveSubsystem m_driveSubsystem;
    // Drive Command
    private final DriveCommand m_driveCommand;

    //----- DRIVETRAIN SHIFTER -----\\

    // Drivetrain Shifter Subsystem
    private final ShifterSubsystem m_shifterSubsystem;
    // Drivetrain Shifter Command
    private final ToggleShifterCommand m_toggleShifterCommand;

    //----- SHOOTER -----\\

    // Indexer subsystem
    private final IndexerMotorSubsystem m_indexerMotorSubsystem;
    // Shooter subsystems
    private final ShooterSubsystem m_shooterSubsystem;
    private final ShooterHoodSubsystem m_shooterHoodSubsystem;

    private final ShootCargoCommand m_shootCargoCommand;

    private final IndexerMotorCommand m_indexerMotorForwardCommand;
    private final IndexerMotorCommand m_indexerMotorReverseCommand;

    private final SequentialCommandGroup m_teleopShootSequence;

    //----- ENDGAME -----\\

    // Endgame Sequence
    private final EndgameManagerCommand m_endgameManagerCommand;

    // Endgame Motors
    private final EndgameMotorSubsystem m_endgameMotorSubsystem;
    private final EndgameArmCommand m_endgameArmCommand;
    private final EndgameArmRevCommand m_endgameArmRevCommand;

    // Endgame Solenoids
    private final EndgamePistonSubsystem m_endgamePiston1;
    private final EndgamePistonSubsystem m_endgamePiston2;
    private final EndgamePistonSubsystem m_endgamePiston3;
    private final EndgamePistonSubsystem m_endgamePiston4;

    //----- LEDS -----\\

    // LED commands

    private final LEDSubsystem m_LEDSubsystem;

    private final LEDCommand m_autonPatternCommand;
    private final LEDCommand m_idlePatternCommand;
    private final LEDCommand m_endgamePatternCommand;

    //----- AUTONOMOUS -----\\

    private final AutoCommandManager m_autoManager;

    //----- VISION -----\\

    UsbCamera m_driverCamera = new UsbCamera("Driver Camera", 0);
    MjpegServer m_mjpegServer = new MjpegServer("Drive Camera Stream", "", 1185);

    //----- PRESSURE MANAGEMENT -----\\

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>RobotContainer</h3>
     * 
     * Manages all robot hardware and software.
     */
    public RobotContainer() {

        /*
         * -----------------------------------------------------------------------------
         * SUBSYSTEM INITIALIZATIONS
         * -----------------------------------------------------------------------------
         */

        //----- LED SUBSYSTEM INITS -----\\

        m_LEDSubsystem = new LEDSubsystem(0);

        //------ PORT FORWARDING ------- \\

        PortForwarder.add(5800, "10.9.30.25", 5800);
        PortForwarder.add(1181, "10.9.30.25", 1181);
        PortForwarder.add(1182, "10.9.30.25", 1182);
        PortForwarder.add(1183, "10.9.30.25", 1183);
        PortForwarder.add(1184, "10.9.30.25", 1184);

        //----- INTAKE SUBSYSTEM INITS -----\\

        m_intakeMotorSubsystem = new IntakeMotorSubsystem(5);
        m_intakePistonSubsystem = new IntakePistonSubsystem(1);

        //----- DRIVETRAIN SUBSYSTEM INITS -----\\

        m_shifterSubsystem = new ShifterSubsystem(0);
        m_driveSubsystem = new DriveSubsystem(1, 8, 2, 7);

        //----- SHOOTER SUBSYSTEM INITS -----\\

        m_indexerMotorSubsystem = new IndexerMotorSubsystem(14, 13);
        m_shooterHoodSubsystem = new ShooterHoodSubsystem(12);
        m_shooterSubsystem = new ShooterSubsystem(10, 6, 11);

        //----- ENDGAME SUBSYSTEM INITS -----\\

        // Endgame Motor Subsystems
        m_endgameMotorSubsystem = new EndgameMotorSubsystem(3, 4);
        // Endgame Piston Subsystems
        m_endgamePiston1 = new EndgamePistonSubsystem(8);
        m_endgamePiston2 = new EndgamePistonSubsystem(9);
        m_endgamePiston3 = new EndgamePistonSubsystem(10);
        m_endgamePiston4 = new EndgamePistonSubsystem(11);

        /*
         * -----------------------------------------------------------------------------
         * CONSTRUCT COMMAND MANAGER
         * -----------------------------------------------------------------------------
         */

        m_autoManager = new AutoCommandManager();
        m_autoManager.addSubsystem(subNames.DriveSubsystem, m_driveSubsystem);
        m_autoManager.addSubsystem(subNames.IntakeMotorSubsystem, m_intakeMotorSubsystem);
        m_autoManager.addSubsystem(subNames.IntakePistonSubsystem, m_intakePistonSubsystem);
        m_autoManager.addSubsystem(subNames.ShooterSubsystem, m_shooterSubsystem);
        m_autoManager.addSubsystem(subNames.ShooterHoodSubsystem, m_shooterHoodSubsystem);
        m_autoManager.addSubsystem(subNames.IndexerMotorSubsystem, m_indexerMotorSubsystem);

        /*
         * -----------------------------------------------------------------------------
         * SENSOR SINGLETON INITIALIZATIONS
         * -----------------------------------------------------------------------------
         */

        // Create instance for sensor singletons-needed for simulation to work properly.
        BallSensorUtility.getInstance();
        EndgameSensorUtility.getInstance();

        /*
         * -----------------------------------------------------------------------------
         * COMMAND INITIALIZATIONS
         * -----------------------------------------------------------------------------
         */

        //----- AUTO COMMAND INITS -----\\

        m_autoManager.initCommands();

        //----- INTAKE COMMAND INITS -----\\
        
        // Intake Motor Commandss
        m_runIntakeMotorsCommand = new RunIntakeMotorsCommand(m_intakeMotorSubsystem, false);
        m_reverseIntakeMotorsCommand = new RunIntakeMotorsCommand(m_intakeMotorSubsystem, true);
        m_stopIntakeMotorsCommand = new StopIntakeMotorsCommand(m_intakeMotorSubsystem);
        // Intake Piston Commands
        m_engageIntakePistonsCommand = new EngageIntakePistonsCommand(m_intakePistonSubsystem);
        m_disengageIntakePistonsCommand = new DisengageIntakePistonsCommand(m_intakePistonSubsystem);

        //----- DRIVETRAIN COMMAND INITS -----\\

        m_driveCommand = new DriveCommand(m_driveSubsystem,m_driverController.getController());

        //----- DRIVETRAIN SHIFTER COMMAND INITS -----\\

        m_toggleShifterCommand = new ToggleShifterCommand(m_shifterSubsystem);

        //----- SHOOTER COMMAND INITS -----\\

        m_shootCargoCommand = new ShootCargoCommand(m_shooterSubsystem, m_indexerMotorSubsystem);

        m_indexerMotorForwardCommand = new IndexerMotorCommand(m_indexerMotorSubsystem, false);
        m_indexerMotorReverseCommand = new IndexerMotorCommand(m_indexerMotorSubsystem, true);

        m_teleopShootSequence = new SequentialCommandGroup(
            new PhotonAimCommand(
                m_driveSubsystem, 
                m_driverController.getController(),
                m_codriverController.getController()
            ),
            new AdjustHoodCommand(m_shooterHoodSubsystem),
            new ShootCargoCommand(
                m_shooterSubsystem, 
                m_indexerMotorSubsystem
            ).withTimeout(ShootCargoCommand.TELEOP_SHOOT_TIME)
        );

        //----- ENDGAME COMMAND INITS -----\\

        // Endgame Arm Commands
        m_endgameArmCommand = new EndgameArmCommand(m_endgameMotorSubsystem);
        m_endgameArmRevCommand = new EndgameArmRevCommand(m_endgameMotorSubsystem);
        m_endgameManagerCommand = new EndgameManagerCommand(
            m_endgameMotorSubsystem,
            m_endgamePiston1, 
            m_endgamePiston2, 
            m_endgamePiston3, 
            m_endgamePiston4,
            m_indexerMotorSubsystem,
            compressor
        );

        //----- SETTING BALL COLOR -----\\

        if (DriverStation.getAlliance() == Alliance.Blue) {
            DriveCameraUtility.getInstance().setBallColor(BallColor.BLUE);
        } else {
            DriveCameraUtility.getInstance().setBallColor(BallColor.RED);

        }

        //----- LED COMMAND INITS-----\\

        m_autonPatternCommand = new LEDCommand(m_LEDSubsystem, m_driverController, LEDPatterns.AutonPattern);
        m_endgamePatternCommand = new LEDCommand(m_LEDSubsystem, m_driverController, LEDPatterns.EndgamePatten);
        m_idlePatternCommand = new LEDCommand(m_LEDSubsystem, m_driverController, LEDPatterns.TeleopIdle);

        // calls the method that configures the buttons
        configureButtonBindings();

        /*
         * -----------------------------------------------------------------------------
         * COMMAND SCHEDULER AND DEFAULTS
         * -----------------------------------------------------------------------------
         */

        // Manages commands via stacking
        CommandScheduler scheduler = CommandScheduler.getInstance();

        // DRIVETRAIN DEFAULTS
        scheduler.setDefaultCommand(m_driveSubsystem, m_driveCommand);

        // INTAKE DEFAULTS
        scheduler.setDefaultCommand(m_intakeMotorSubsystem, m_stopIntakeMotorsCommand);
        scheduler.setDefaultCommand(m_intakePistonSubsystem, m_disengageIntakePistonsCommand);

        // ENDGAME DEFAULTS
        // scheduler.setDefaultCommand(endgameMotorSubsystem, new
        // EndgameRotateHorizonalCommand(endgameMotorSubsystem)); // -GET ENCODER
        // WORKING
        scheduler.setDefaultCommand(m_endgamePiston1, new EndgameCloseClawCommand(m_endgamePiston1));
        scheduler.setDefaultCommand(m_endgamePiston2, new EndgameCloseClawCommand(m_endgamePiston2));
        scheduler.setDefaultCommand(m_endgamePiston3, new EndgameCloseClawCommand(m_endgamePiston3));
        scheduler.setDefaultCommand(m_endgamePiston4, new EndgameCloseClawCommand(m_endgamePiston4));
        scheduler.setDefaultCommand(m_indexerMotorSubsystem, m_indexerMotorForwardCommand);

        compressor.enableAnalog(100, 115);
        System.out.println("COMPRESSOR VALUE: " + compressor.getPressure());
    }

    /**
     * <h3>configureButtonBindings</h3> 
     * 
     * contains the controller binds and button Bindings
     * defines button->command mappings
     * 
     */
    private void configureButtonBindings() {

        //----- DRIVER CONTROLLER -----\\

        // Shifts the drivetrain when shifter trigger is pulled
        m_driverController.getRightTrigger().whileActiveOnce(
            m_toggleShifterCommand
        );

        m_driverController.getLeftBumper().whileActiveOnce(
            m_teleopShootSequence
        );

        m_driverController.getRightBumper().whileActiveOnce(
            m_shootCargoCommand
        );

        //----- CODRIVER CONTROLLER -----\\

        // Checks if LB is pressed, then it will engage the intake pistons
        m_codriverController.getLeftBumper().whileActiveOnce(
            m_engageIntakePistonsCommand
        );

        // Checks if LB is pressed and B isn't pressed, then it will run intake
        m_codriverController.getLeftBumper().and(m_codriverController.getBButton().negate()).whileActiveOnce(
            m_runIntakeMotorsCommand
        );

        // Checks if LB and B is pressed, then it will reverse the intake
        m_codriverController.getLeftBumper().and(m_codriverController.getBButton()).whileActiveOnce(
            new ParallelCommandGroup(
                m_reverseIntakeMotorsCommand,
                m_indexerMotorReverseCommand
            )
        );

        m_codriverController.getRightBumper().whileActiveOnce(
            m_teleopShootSequence
        );

        // Manually rotates the endgame arms while pressed
        m_codriverController.getYButton().whileActiveOnce(
            m_endgameArmCommand
        );

        // Manually rotates the endgame arms in reverse while pressed
        m_codriverController.getAButton().whileActiveOnce(
            m_endgameArmRevCommand
        );

        m_codriverController.getPOVLeftTrigger().whileActiveOnce(
            new ParallelCommandGroup(
                new AdjustHoodCommand(
                    m_shooterHoodSubsystem,
                    ShooterUtility.calculateHoodPos(9)
                ),
                new ShootCargoCommand(
                    m_shooterSubsystem, 
                    m_indexerMotorSubsystem,
                    ShooterUtility.calculateTopSpeed(9),
                    ShooterUtility.calculateBottomSpeed(9)
                )
            ).withTimeout(0.1)
        );

        m_codriverController.getPOVUpTrigger().whileActiveOnce(
            new ParallelCommandGroup(
                new AdjustHoodCommand(
                    m_shooterHoodSubsystem,
                    ShooterUtility.calculateHoodPos(16.5)
                ),
                new ShootCargoCommand(
                    m_shooterSubsystem, 
                    m_indexerMotorSubsystem,
                    ShooterUtility.calculateTopSpeed(16.5),
                    ShooterUtility.calculateBottomSpeed(16.5)
                )
            ).withTimeout(0.1)
        );

        m_codriverController.getPOVDownTrigger().whileActiveOnce(
            new ParallelCommandGroup(
                new AdjustHoodCommand(
                    m_shooterHoodSubsystem,
                    ShooterUtility.calculateHoodPos(19 / 12)
                ),
                new ShootCargoCommand(
                    m_shooterSubsystem, 
                    m_indexerMotorSubsystem,
                    ShooterUtility.calculateTopSpeed(19 / 12),
                    ShooterUtility.calculateBottomSpeed(19 / 12)
                )
            ).withTimeout(0.1)
        );

        m_codriverController.getStartButton().whileActiveOnce(
            new SequentialCommandGroup(
                new AdjustHoodCommand(
                    m_shooterHoodSubsystem, 
                    -1
                ),
                new InstantCommand(() -> {
                    PhotonVisionUtility.getInstance().getHubTrackingCamera().setDriverMode(true);
                }),
                new ParallelCommandGroup(
                    m_endgameManagerCommand, 
                    m_endgamePatternCommand
                )
            )
        );
        // .whenInactive(new InstantCommand(() -> {
        //     PhotonVisionUtility.getInstance().getHubTrackingCamera().setLED(VisionLEDMode.kOn);
        // }));

        m_codriverController.getBackButton().whileActiveOnce(
            new SequentialCommandGroup(
                new InstantCommand(m_endgameManagerCommand::resetState, m_endgameMotorSubsystem),
                new EndgameRotateArmCommand(m_endgameMotorSubsystem, EndgamePosition.ResetPosition)));

        /*
         * codriverController.getXButton().whileActiveOnce(new
         * InstantCommand(shooterHoodSubsystem::setSlowSpeed, shooterHoodSubsystem));
         * codriverController.getXButton().negate().whileActiveOnce(new
         * InstantCommand(shooterHoodSubsystem::stopHood, shooterHoodSubsystem));
         * codriverController.getBButton().whileActiveOnce(new
         * InstantCommand(shooterHoodSubsystem::setSlowRevSpeed, shooterHoodSubsystem));
         * codriverController.getBButton().negate().whileActiveOnce(new
         * InstantCommand(shooterHoodSubsystem::stopHood, shooterHoodSubsystem));
         */
    }

    /**
     * <h3>beginTeleopRunCommands</h3>
     * 
     * Runs when the robot is enabled in teleop mode.
     * 
     * This gets the command scheduler and sets up buttons
     */
    public void beginTeleopRunCommands() {

        // Sets the brake mode to coast
        m_driveSubsystem.setMotorBrakeMode(NeutralMode.Brake);


        rescheduleAutonomousLEDs(false);

        PhotonVisionUtility.getInstance().setPiCameraExposure();
    }

    /**
     * <h3>startCamera</h3>
     * 
     * Starts up the driver camera.
     */
    public void startCamera() {

        // Set the video mode for the camera. This will tell the camera that we want a
        // color stream with resolution 160x120
        m_driverCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 15);

        // Set the source of the stream to the USB camera
        m_mjpegServer.setSource(m_driverCamera);
        // Set the compression. This gives us an OK quality stream while not chewing
        // bandwidth
        m_mjpegServer.setCompression(70);

        // Get the network table instance
        var currentNTInstance = NetworkTableInstance.getDefault();
        // Get the camera publisher table so we can write our camera data
        var cameraPublisherTable = currentNTInstance.getTable("CameraPublisher");
        // Set our camera data in the publisher table
        // This will cause the camera server to recognize the stream
        cameraPublisherTable.getEntry("Driver Camera/connected").setBoolean(true);
        cameraPublisherTable.getEntry("Driver Camera/description").setString("Driver Camera");
        cameraPublisherTable.getEntry("Driver Camera/streams")
                .setStringArray(new String[] { "mjpg:http://10.9.30.2:1185/?action=stream",
                        "mjpg:http://172.22.11.2:1185/?action=stream" });

    }

    /**
     * <h3>getAutonomousCommand</h3>
     * 
     * Gets the selected autonomous command.
     */
    public Command getAutonomousCommand() {
        return m_autoManager.getAutonomousCommand();
    }

    /**
     * <h3>beginAutoRunCommands</h3>
     * 
     * Prepares the robot for autonomous.
     */
    public void beginAutoRunCommands() {
        // Sets the brake mode to brake
        m_driveSubsystem.setMotorBrakeMode(NeutralMode.Brake);
        rescheduleAutonomousLEDs(true);

        PhotonVisionUtility.getInstance().setPiCameraExposure();
    }

    /**
     * <h3>testInit</h3>
     * 
     * Initializes the robot for test mode.
     */
    public void testInit() {
        stopSubsystems();
        m_endgameMotorSubsystem.refollowEndgameMotors();
    }

    /**
     * <h3>testPeriodic</h3>
     * 
     * Test mode periodic.
     */
    public void testPeriodic() {
        // NEED REFOLLOW TO KEEP MASTER-SLAVE PAIR WORKING
        m_endgameMotorSubsystem.refollowEndgameMotors();
        
        if (m_driverController.getLeftBumper().get()) {
            if (m_driverController.getYButton().get()) {
                m_endgamePiston1.open();
            } else {
                m_endgamePiston1.closed();
            }
            if (m_driverController.getAButton().get()) {
                m_endgamePiston2.open();
            } else {
                m_endgamePiston2.closed();
            }
            if (m_driverController.getBButton().get()) {
                m_endgamePiston3.open();
            } else {
                m_endgamePiston3.closed();
            }
            if (m_driverController.getXButton().get()) {
                m_endgamePiston4.open();
            } else {
                m_endgamePiston4.closed();
            }
        } else {
            if (m_driverController.getYButton().get()) {
                m_endgameMotorSubsystem.setMotorSpeed(0.2);
            } else if (m_driverController.getAButton().get()) {
                m_endgameMotorSubsystem.setMotorSpeed(-0.2);
            } else {
                m_endgameMotorSubsystem.setMotorSpeed(0.0);
            }
        }
    }

    /**
     * <h3>testExit</h3>
     * 
     * Runs when robot exits test mode.
     */
    public void testExit() {
        m_driveSubsystem.refollowDriveMotors();
        m_shooterSubsystem.refollowShooterMotors();
        m_endgameMotorSubsystem.refollowEndgameMotors();
    }

    /**
     * <h3>stopSubsystems</h3>
     * 
     * Sets the intake and drive motors to 0.
     */
    public void stopSubsystems() {
        m_intakeMotorSubsystem.setMotorSpeed(0.0);
        m_driveSubsystem.setVoltages(0.0, 0.0);
        m_indexerMotorSubsystem.stopMotors();
    }

    //----- ROBOT SIMULATION -----\\

    private final RamseteController m_ramsete = new RamseteController();
    private final SimulatedDrivetrain m_simDrive = new SimulatedDrivetrain();

    private final Timer m_timer = new Timer();

    private Trajectory m_trajectory;
    private Command m_autocmd = null;

    // This begins the robot sim and sets our a trajectory and paths.
    /**
     * <h3>robotSimInit</h3>
     * 
     * Runs when the robot is initialized in simulation.
     */
    public void robotSimInit() {
        /*
         * String trajectoryJSON = Filesystem.getDeployDirectory() +
         * "/Paths/Bounce.wpilib.json";
         * //Tries to create a trajectory from a JSON file. Logs and throws exception if
         * fails.
         * try {
         * //Path trajectoryPath =
         * Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         * Path trajectoryPath =
         * Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
         * m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         * } catch (IOException ex) {
         * DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
         * ex.getStackTrace());
         * throw new RuntimeException("Unable to open trajectory: " + trajectoryJSON);
         * }
         */
    }

    /**
     * <h3>autoSimInit</h3>
     * 
     * Begins autonomous simulation. Resets position and timer.
     */
    public void autoSimInit() {
        rescheduleAutonomousLEDs(true);
        m_autocmd = m_autoManager.getAutonomousCommand();
        if (m_autocmd != null) {
            if (m_autocmd instanceof PathPlannerSequentialCommandGroupUtility) {
                List<Trajectory> list = ((PathPlannerSequentialCommandGroupUtility) m_autocmd).getTrajectories();
                if (list.size() == 0) {
                    DriverStation.reportError(
                            "Missing trajectories in (please add them this.addTrajectory()): " + m_autocmd.toString(),
                            true);
                    throw new RuntimeException(
                            "Missing trajectories in (please add them this.addTrajectory()): " + m_autocmd.toString());
                }
                int ii = 0;
                for (Trajectory traj : list) {
                    if (ii == 0) {
                        m_trajectory = traj;
                    } else {
                        m_trajectory = m_trajectory.concatenate(traj);
                    }
                    ii++;
                }
            } else {
                DriverStation.reportError("Unable to open trajectory: " + m_autocmd.toString(), true);
                throw new RuntimeException("Unable to open trajectory: " + m_autocmd.toString());
            }
            m_timer.reset();
            m_timer.start();
            m_simDrive.resetOdometry(m_trajectory.getInitialPose());
        }
    }

    private void rescheduleAutonomousLEDs(boolean useAutonomousLEDCmd) {
        LEDCommand ledCommand = (useAutonomousLEDCmd) ? m_autonPatternCommand :
            m_idlePatternCommand;
        LEDCommand endledCommand = (useAutonomousLEDCmd) ? m_idlePatternCommand :
        m_autonPatternCommand;
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.unregisterSubsystem(m_LEDSubsystem);
        endledCommand.cancel();
        scheduler.setDefaultCommand(m_LEDSubsystem, ledCommand);
    }

    /**
     * <h3>robotSimPeriodic</h3>
     * 
     * Updates simulated robot periodically.
     */
    public void robotSimPeriodic() {
        if (m_autocmd != null) {
            m_simDrive.periodic();
        }
    }

    /**
     * <h3>autoSimPeriodic</h3>
     * 
     * Updates the position of the simulated robot autonomous periodically.
     */
    public void autoSimPeriodic() {
        if (m_autocmd != null) {
            double elapsed = m_timer.get();
            Trajectory.State reference = m_trajectory.sample(elapsed);
            ChassisSpeeds speeds = m_ramsete.calculate(m_simDrive.getPose(), reference);
            m_simDrive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
    }

    /**
     * <h3>simPeriodic</h3>
     * 
     * Updates simulation
     */
    public void simPeriodic() {
        if (m_autocmd != null) {
            m_simDrive.simulationPeriodic();
        }
    }

    //----- DISABLED -----\\

    /**
     * <h3>disabledInit</h3>
     * 
     * Robot initialization when disabled.
     */
    public void disabledInit() {
        m_idlePatternCommand.solidYellowLEDs();
    }

} // End of RobotContainer