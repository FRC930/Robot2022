package frc.robot;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.List;
import java.util.Scanner;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.photonvision.common.hardware.VisionLEDMode;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerForwardCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LEDCommand.LEDPatterns;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.commands.autocommands.AutoCommandManager;
import frc.robot.commands.autocommands.AutoCommandManager.subNames;
import frc.robot.commands.autovisioncommands.HubAimCommand;
import frc.robot.commands.autovisioncommands.PhotonAimCommand;
import frc.robot.commands.autovisioncommands.PigeonAimCommand;
import frc.robot.commands.endgamecommands.EndgameArmCommand;
import frc.robot.commands.endgamecommands.EndgameArmRevCommand;
import frc.robot.commands.endgamecommands.EndgameCloseClawCommand;
import frc.robot.commands.endgamecommands.EndgameManagerCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.DisengageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.StopIntakeMotorsCommand;
import frc.robot.commands.shootercommands.ShootCargoCommand;
import frc.robot.commands.shootercommands.AdjustHoodCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.utilities.SimulatedDrivetrain;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.DriveCameraUtility.BallColor;
import frc.robot.utilities.EndgameSensorUtility;
import frc.robot.utilities.PhotonVisionUtility;
import frc.robot.utilities.ShuffleboardUtility;

public class RobotContainer {

    // ----- XBOX CONTROLLER(S) -----\\

    // Driver Controller
    private final ControllerManager driverController = new ControllerManager(0);
    // Codriver Controller
    private final ControllerManager codriverController = new ControllerManager(1);
    /*
     * // Left Joystick
     * public static final int XB_AXIS_LEFT_X = 0;
     * public static final int XB_AXIS_LEFT_Y = 1;
     * // Triggers
     * public static final int XB_AXIS_LT = 2;
     * public static final int XB_AXIS_RT = 3;
     * // Right Joystick
     * public static final int XB_AXIS_RIGHT_X = 4;
     * public static final int XB_AXIS_RIGHT_Y = 5;
     * 
     * // Buttons
     * public static final int XB_A = 1;
     * public static final int XB_B = 2;
     * public static final int XB_X = 3;
     * public static final int XB_Y = 4;
     * public static final int XB_LB = 5;
     * public static final int XB_RB = 6;
     * public static final int XB_BACK = 7;
     * public static final int XB_START = 8;
     * public static final int XB_LEFTSTICK_BUTTON = 9;
     * public static final int XB_RIGHTSTICK_BUTTON = 10;
     */

    // ----- CAMERA CONSTANTS -----\\

    public static final int CAMERA_WIDTH = 160;
    public static final int CAMERA_HEIGHT = 120;
    public static final int CAMERA_FPS = 30;

    // ----- INTAKE -----\\

    // Intake Motor Subsystem
    private final IntakeMotorSubsystem intakeMotorSubsystem;
    // Intake Piston Subsystem
    private final IntakePistonSubsystem intakePistonSubsystem;

    // Intake Motor Commands
    private final RunIntakeMotorsCommand runIntakeMotorsCommand;
    private final RunIntakeMotorsCommand reverseIntakeMotorsCommand;
    private final StopIntakeMotorsCommand stopIntakeMotorsCommand;

    // Intake Piston Commands
    private final EngageIntakePistonsCommand engageIntakePistonsCommand;
    private final DisengageIntakePistonsCommand disengageIntakePistonsCommand;

    // ----- DRIVETRAIN -----\\

    // Drive Subsystem
    private final DriveSubsystem driveSubsystem;
    // Drive Command
    private final DriveCommand driveCommand;

    // ----- DRIVETRAIN SHIFTER -----\\

    // Drivetrain Shifter Subsystem
    private final ShifterSubsystem shifterSubsystem;
    // Drivetrain Shifter Command
    private final ToggleShifterCommand toggleShifterCommand;

    // ----- SHOOTER -----\\

    // Indexer subsystem
    private final IndexerMotorSubsystem indexerMotorSubsystem;
    // Shooter subsystems
    private final FlywheelSubsystem flywheelSubsystem;
    private final ShooterHoodSubsystem shooterHoodSubsystem;

    // ----- ENDGAME -----\\
    private final EndgameManagerCommand endgameManager;
    // Endgame Arm Commands
    private final EndgameMotorSubsystem endgameMotorSubsystem;
    private final EndgameArmCommand endgameArmCommand;
    private final EndgameArmRevCommand endgameArmRevCommand;

    // Endgame Piston Subsystems
    private final EndgamePistonSubsystem endgamePiston1;
    private final EndgamePistonSubsystem endgamePiston2;
    private final EndgamePistonSubsystem endgamePiston3;
    private final EndgamePistonSubsystem endgamePiston4;

    private final PhotonAimCommand photonAimCommand;
    private final PigeonAimCommand pigeonAimCommand;

    // LED commands
    private final LEDCommand autonPatternCommand;
    private final LEDCommand idlePatternCommand;
    private final LEDCommand endgamePatternCommand;

    private final LEDSubsystem ledSubsystem;
    // ----- AUTONOMOUS -----\\
    private final AutoCommandManager autoManager;

    UsbCamera driverCamera = new UsbCamera("Driver Camera", 0);
    MjpegServer mjpegServer = new MjpegServer("Drive Camera Stream", "", 1185);

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

    // ----- CONSTRUCTOR -----\\

    /**
     * <h3>RobotContainer</h3>
     * 
     * Initializes the robot
     */
    public RobotContainer() {
        /*
         * -----------------------------------------------------------------------------
         * ---
         * SUBSYSTEM INITIALIZATIONS
         * -----------------------------------------------------------------------------
         * ---
         */

        // ----- LED SUBSYSTEM INITS -----\\
        ledSubsystem = new LEDSubsystem(0);

        // ------ PORT FORWARDING ------- \\
        PortForwarder.add(5800, "10.9.30.25", 5800);
        PortForwarder.add(1181, "10.9.30.25", 1181);
        PortForwarder.add(1182, "10.9.30.25", 1182);
        PortForwarder.add(1183, "10.9.30.25", 1183);
        PortForwarder.add(1184, "10.9.30.25", 1184);

        // ----- INTAKE SUBSYSTEM INITS -----\\
        intakeMotorSubsystem = new IntakeMotorSubsystem(5);
        intakePistonSubsystem = new IntakePistonSubsystem(1, 12);

        // ----- DRIVETRAIN SUBSYSTEM INITS -----\\
        shifterSubsystem = new ShifterSubsystem(0);
        driveSubsystem = new DriveSubsystem(1, 8, 2, 7);

        // ----- SHOOTER SUBSYSTEM INITS -----\\
        indexerMotorSubsystem = new IndexerMotorSubsystem(14, 13);
        shooterHoodSubsystem = new ShooterHoodSubsystem(12);
        flywheelSubsystem = new FlywheelSubsystem(11, 10, 6);

        // ----- ENDGAME SUBSYSTEM INITS -----\\
        // Endgame Motor Subsystems
        endgameMotorSubsystem = new EndgameMotorSubsystem(3, 4);
        // Endgame Piston Subsystems
        endgamePiston1 = new EndgamePistonSubsystem(8);
        endgamePiston2 = new EndgamePistonSubsystem(9);
        endgamePiston3 = new EndgamePistonSubsystem(10);
        endgamePiston4 = new EndgamePistonSubsystem(11);

        /*
         * -----------------------------------------------------------------------------
         * ---
         * CONSTRUCT COMMAND MANAGER
         * -----------------------------------------------------------------------------
         * ---
         */
        autoManager = new AutoCommandManager();
        autoManager.addSubsystem(subNames.DriveSubsystem, driveSubsystem);
        autoManager.addSubsystem(subNames.IntakeMotorSubsystem, intakeMotorSubsystem);
        autoManager.addSubsystem(subNames.IntakePistonSubsystem, intakePistonSubsystem);
        autoManager.addSubsystem(subNames.FlywheelSubsystem, flywheelSubsystem);
        autoManager.addSubsystem(subNames.IndexerMotorSubsystem, indexerMotorSubsystem);

        // Create instance for sensor singletons-needed for simulation to work properly.
        BallSensorUtility.getInstance();
        EndgameSensorUtility.getInstance();

        /*
         * -----------------------------------------------------------------------------
         * ---
         * COMMAND INITIALIZATIONS
         * -----------------------------------------------------------------------------
         * ---
         */

        // ----- AUTO COMMAND INITS -----\\
        autoManager.initCommands();

        // ----- INTAKE COMMAND INITS -----\\
        // Intake Motor Commandss
        runIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeMotorSubsystem, false);
        reverseIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeMotorSubsystem, true);
        stopIntakeMotorsCommand = new StopIntakeMotorsCommand(intakeMotorSubsystem);
        // Intake Piston Commands
        engageIntakePistonsCommand = new EngageIntakePistonsCommand(intakePistonSubsystem);
        disengageIntakePistonsCommand = new DisengageIntakePistonsCommand(intakePistonSubsystem);

        // ----- DRIVETRAIN COMMAND INITS -----\\
        driveCommand = new DriveCommand(
                driveSubsystem,
                driverController.getController());

        // ----- DRIVETRAIN SHIFTER COMMAND INITS -----\\
        toggleShifterCommand = new ToggleShifterCommand(shifterSubsystem);

        // ----- ENDGAME COMMAND INITS -----\\
        // Endgame Arm Commands
        endgameArmCommand = new EndgameArmCommand(endgameMotorSubsystem);
        endgameArmRevCommand = new EndgameArmRevCommand(endgameMotorSubsystem);
        endgameManager = new EndgameManagerCommand(endgameMotorSubsystem,
                endgamePiston1, endgamePiston2, endgamePiston3, endgamePiston4);

        pigeonAimCommand = new PigeonAimCommand(driveSubsystem);
        photonAimCommand = new PhotonAimCommand(driveSubsystem, driverController.getController(),
                codriverController.getController());

        // ----- SETTING BALL COLOR -----\\
        if (DriverStation.getAlliance() == Alliance.Blue) {
            DriveCameraUtility.getInstance().setBallColor(BallColor.BLUE);
        } else {
            DriveCameraUtility.getInstance().setBallColor(BallColor.RED);

        }

        // ----- LED COMMAND INITS-----\\
        autonPatternCommand = new LEDCommand(ledSubsystem, LEDPatterns.AutonPattern);
        endgamePatternCommand = new LEDCommand(ledSubsystem, LEDPatterns.EndgamePatten);
        idlePatternCommand = new LEDCommand(ledSubsystem, LEDPatterns.TeleopIdle);

        // calls the method that configures the buttons
        configureButtonBindings();

        // Manages commands via stacking
        CommandScheduler scheduler = CommandScheduler.getInstance();

        // DRIVETRAIN DEFAULTS
        scheduler.setDefaultCommand(driveSubsystem, driveCommand);

        // INTAKE DEFAULTS
        scheduler.setDefaultCommand(intakeMotorSubsystem, stopIntakeMotorsCommand);
        scheduler.setDefaultCommand(intakePistonSubsystem, disengageIntakePistonsCommand);

        // ENDGAME DEFAULTS
        // scheduler.setDefaultCommand(endgameMotorSubsystem, new
        // EndgameRotateHorizonalCommand(endgameMotorSubsystem)); // -GET ENCODER
        // WORKING
        scheduler.setDefaultCommand(endgamePiston1, new EndgameCloseClawCommand(endgamePiston1));
        scheduler.setDefaultCommand(endgamePiston2, new EndgameCloseClawCommand(endgamePiston2));
        scheduler.setDefaultCommand(endgamePiston3, new EndgameCloseClawCommand(endgamePiston3));
        scheduler.setDefaultCommand(endgamePiston4, new EndgameCloseClawCommand(endgamePiston4));
        // scheduler.setDefaultCommand(indexerMotorSubsystem, new
        // IndexerForwardCommand(indexerMotorSubsystem, false));;

        compressor.enableAnalog(100, 115);

        if (Robot.isReal()) {
            try {
                BufferedInputStream zipFile = new BufferedInputStream(
                        new URL("http://10.9.30.25:5800/api/settings/photonvision_config.zip").openStream());

                ObjectMapper objectMapper = new ObjectMapper();

                File destDir = new File("settings_unzipped");
                byte[] buffer = new byte[1024];
                ZipInputStream zis = new ZipInputStream(zipFile);
                ZipEntry zipEntry = zis.getNextEntry();
                while (zipEntry != null) {
                    File newFile = newFile(destDir, zipEntry);
                    if (zipEntry.isDirectory()) {
                        if (!newFile.isDirectory() && !newFile.mkdirs()) {
                            throw new IOException("Failed to create directory " + newFile);
                        }
                    } else {
                        File parent = newFile.getParentFile();
                        if (!parent.isDirectory() && !parent.mkdirs()) {
                            throw new IOException("Failed to create directory " + parent);
                        }

                        FileOutputStream fos = new FileOutputStream(newFile);
                        int len;
                        while ((len = zis.read(buffer)) > 0) {
                            fos.write(buffer, 0, len);
                        }
                        fos.close();
                    }
                    zipEntry = zis.getNextEntry();
                }
                zis.closeEntry();
                zis.close();

                File pipelinesDir = new File("settings_unzipped/cameras/Integrated_Camera/pipelines"); // mmal_service_16.1
                String[] pipelinesList = pipelinesDir.list();
                for (String piplineFile : pipelinesList) {
                    File currentPipeline = new File(pipelinesDir.getPath() + "/" + piplineFile);
                    String fileContents = "";
                    Scanner sc = new Scanner(currentPipeline);
                    while (sc.hasNextLine()) {
                        fileContents += sc.nextLine() + "\n";
                    }
                    sc.close();

                    JsonNode jsonNode = objectMapper.readTree(fileContents);
                    JsonNode settingsMap = jsonNode.get(1);
                    String pipelineName = settingsMap.get("pipelineNickname").asText();
                    int pipelineIndex = Integer.parseInt(settingsMap.get("pipelineIndex").asText());
                    double exposure = Double.parseDouble(settingsMap.get("cameraExposure").asText());

                    ShuffleboardUtility.getInstance().addPipelineChooser(pipelineName, pipelineIndex);
                    PhotonVisionUtility.getInstance().addPipeline(pipelineName, exposure);

                    System.out.println(pipelineName);
                    System.out.println(exposure);
                }
            } catch (MalformedURLException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private static File newFile(File destinationDir, ZipEntry zipEntry) throws IOException {
        File destFile = new File(destinationDir, zipEntry.getName());

        String destDirPath = destinationDir.getCanonicalPath();
        String destFilePath = destFile.getCanonicalPath();

        if (!destFilePath.startsWith(destDirPath + File.separator)) {
            throw new IOException("Entry is outside of the target dir: " + zipEntry.getName());
        }

        return destFile;
    }

    /**
     * configureButtonBindings
     * contains the controller binds and button Bindings
     * defines button->command mappings
     * 
     */
    private void configureButtonBindings() {
        // DRIVER CONTROLLER BINDS
        /*
         * AxisTrigger shifterTrigger = new AxisTrigger(driverController, XB_AXIS_RT);
         * 
         * JoystickButton launchButton = new JoystickButton(driverController, XB_LB);
         * JoystickButton rotateArmButton = new JoystickButton(driverController, XB_Y);
         * JoystickButton rotateArmRevButton = new JoystickButton(driverController,
         * XB_A);
         * JoystickButton endgameSensorCloseButton = new
         * JoystickButton(driverController, XB_X);
         * JoystickButton rotateUntilTouchingButton = new
         * JoystickButton(driverController, XB_B);
         * JoystickButton endgameComplete = new JoystickButton(driverController,
         * XB_START);
         * JoystickButton indexerButton = new JoystickButton(codriverController, XB_RB);
         * 
         * // CODRIVER CONTROLLER BINDS
         * JoystickButton intakeButton = new JoystickButton(codriverController, XB_LB);
         * JoystickButton reverseIntakeButton = new JoystickButton(codriverController,
         * XB_B);
         */

        // Shifts the drivetrain when shifter trigger is pulled
        driverController.getRightTrigger().whileActiveOnce(toggleShifterCommand);

        // Checks if LB is pressed, then it will engage the intake pistons
        codriverController.getLeftBumper().whileActiveOnce(
                new ParallelCommandGroup(engageIntakePistonsCommand));
        // Checks if LB is pressed and B isn't pressed, then it will run intake
        codriverController.getLeftBumper().and(
                codriverController.getBButton().negate()).whileActiveOnce(runIntakeMotorsCommand);
        // Checks if LB and B is pressed, then it will reverse the intake
        codriverController.getLeftBumper().and(codriverController.getBButton())
                .whileActiveOnce(new ParallelCommandGroup(reverseIntakeMotorsCommand,
                        new IndexerForwardCommand(indexerMotorSubsystem, true)));

        // Manually rotates the endgame arms while pressed
        codriverController.getYButton().whileActiveOnce(endgameArmCommand);

        // Manually rotates the endgame arms in reverse while pressed
        codriverController.getAButton().whileActiveOnce(endgameArmRevCommand);

        codriverController.getStartButton()
                .whileActiveOnce(new ParallelCommandGroup(endgameManager, endgamePatternCommand));
        driverController.getLeftBumper()
                .whileActiveOnce(new SequentialCommandGroup(new HubAimCommand(driveSubsystem),
                        new AdjustHoodCommand(shooterHoodSubsystem),
                        new ShootCargoCommand(flywheelSubsystem, indexerMotorSubsystem)
                                .withTimeout(ShootCargoCommand.SHOOT_TIME)));
        driverController.getLeftTrigger().whileActiveOnce(new IndexerForwardCommand(indexerMotorSubsystem, false));
        driverController.getRightBumper()
                .whileActiveOnce(new ShootCargoCommand(flywheelSubsystem, indexerMotorSubsystem));
        driverController.getPOVUpTrigger().whileActiveOnce(new AdjustHoodCommand(shooterHoodSubsystem, 28.44));
        driverController.getPOVLeftTrigger().whileActiveOnce(new AdjustHoodCommand(shooterHoodSubsystem));
        driverController.getPOVDownTrigger().whileActiveOnce(new AdjustHoodCommand(shooterHoodSubsystem, 0.0));
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
     * <p>
     * Runs when the robot is enabled in teleop mode.
     * </p>
     * <p>
     * This gets the command scheduler and sets up buttons
     * </p>
     */
    public void beginTeleopRunCommands() {
        // Sets the brake mode to coast
        driveSubsystem.setMotorBrakeMode(NeutralMode.Brake);
        // rescheduleAutonomousLEDs(false);
        CommandScheduler.getInstance().unregisterSubsystem(ledSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(ledSubsystem, idlePatternCommand);

        PhotonVisionUtility.getInstance().getHubTrackingCamera().setDriverMode(false);
        PhotonVisionUtility.getInstance().getHubTrackingCamera().setLED(VisionLEDMode.kOff);

        PhotonVisionUtility.getInstance().setPiCameraExposure();
    }

    public void startCamera() {
        // Set the video mode for the camera. This will tell the camera that we want a
        // color stream with resolution 160x120
        driverCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 30);

        // Set the source of the stream to the USB camera
        mjpegServer.setSource(driverCamera);
        // Set the compression. This gives us an OK quality stream while not chewing
        // bandwidth
        mjpegServer.setCompression(37);

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
    *
    */
    public Command getAutonomousCommand() {
        return autoManager.getAutonomousCommand();
    }

    public void beginAutoRunCommands() {
        // Sets the brake mode to brake
        driveSubsystem.setMotorBrakeMode(NeutralMode.Brake);
        // rescheduleAutonomousLEDs(true);
        CommandScheduler.getInstance().unregisterSubsystem(ledSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(ledSubsystem, autonPatternCommand);

        PhotonVisionUtility.getInstance().getHubTrackingCamera().setDriverMode(false);
        PhotonVisionUtility.getInstance().getHubTrackingCamera().setLED(VisionLEDMode.kOff);

        PhotonVisionUtility.getInstance().setPiCameraExposure();
    }

    public void testInit() {
        stopSubsystems();
        endgameMotorSubsystem.refollowEndgameMotors();
    }

    public void testPeriodic() {
        // TODO: figure out why we need this-need to repair
        endgameMotorSubsystem.refollowEndgameMotors();
        if (driverController.getLeftBumper().get()) {
            if (driverController.getYButton().get()) {
                endgamePiston1.open();
            } else {
                endgamePiston1.closed();
            }
            if (driverController.getAButton().get()) {
                endgamePiston2.open();
            } else {
                endgamePiston2.closed();
            }
            if (driverController.getBButton().get()) {
                endgamePiston3.open();
            } else {
                endgamePiston3.closed();
            }
            if (driverController.getXButton().get()) {
                endgamePiston4.open();
            } else {
                endgamePiston4.closed();
            }
        } else {
            if (driverController.getYButton().get()) {
                endgameMotorSubsystem.setMotorSpeed(0.2);
            } else if (driverController.getAButton().get()) {
                endgameMotorSubsystem.setMotorSpeed(-0.2);
            } else {
                endgameMotorSubsystem.setMotorSpeed(0.0);
            }
        }
    }

    public void testExit() {
        driveSubsystem.refollowDriveMotors();
        endgameMotorSubsystem.refollowEndgameMotors();
    }

    public void stopSubsystems() {
        intakeMotorSubsystem.setMotorSpeed(0.0);
        driveSubsystem.setVoltages(0.0, 0.0);
    }

    private final RamseteController m_ramsete = new RamseteController();
    private final SimulatedDrivetrain m_simDrive = new SimulatedDrivetrain();

    private final Timer m_timer = new Timer();

    private Trajectory m_trajectory;
    private Command m_autocmd = null;

    // This begins the robot sim and sets our a trajectory and paths.
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

    // Begins autonomous simulation. Resets position and timer.
    public void autoSimInit() {
        // rescheduleAutonomousLEDs(true);
        m_autocmd = autoManager.getAutonomousCommand();
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

    // private void rescheduleAutonomousLEDs(boolean useAutonomousLEDCmd) {
    // LEDCommand ledCommand = (useAutonomousLEDCmd) ? autonPatternCommand :
    // idlePatternCommand;
    // LEDCommand endledCommand = (useAutonomousLEDCmd) ? idlePatternCommand :
    // autonPatternCommand;
    // CommandScheduler scheduler = CommandScheduler.getInstance();
    // scheduler.unregisterSubsystem(ledSubsystem);
    // endledCommand.cancel();
    // scheduler.setDefaultCommand(ledSubsystem, ledCommand);
    // }

    // Updates simulated robot periodically.
    public void robotSimPeriodic() {
        if (m_autocmd != null) {
            m_simDrive.periodic();
        }
    }

    // Updates the position of the simulated robot autonomous periodically.
    public void autoSimPeriodic() {
        if (m_autocmd != null) {
            double elapsed = m_timer.get();
            Trajectory.State reference = m_trajectory.sample(elapsed);
            ChassisSpeeds speeds = m_ramsete.calculate(m_simDrive.getPose(), reference);
            m_simDrive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
    }

    // Updates simulation
    public void simPeriodic() {
        if (m_autocmd != null) {
            m_simDrive.simulationPeriodic();
        }
    }

    public void disabledInit() {
        idlePatternCommand.solidYellowLEDs();
        PhotonVisionUtility.getInstance().getHubTrackingCamera().setDriverMode(true);
        PhotonVisionUtility.getInstance().getHubTrackingCamera().setLED(VisionLEDMode.kDefault);
    }
} // End of RobotContainer