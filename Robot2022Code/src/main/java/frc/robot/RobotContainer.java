package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerForwardCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.commands.LEDCommand.LEDPatterns;
import frc.robot.commands.autocommands.AutoCommandManager;
import frc.robot.commands.autocommands.AutoCommandManager.subNames;
import frc.robot.commands.autovisioncommands.HubAimingCommand;
import frc.robot.commands.endgamecommands.EndgameArmCommand;
import frc.robot.commands.endgamecommands.EndgameArmRevCommand;
import frc.robot.commands.endgamecommands.EndgameCloseClawSingleCommand;
import frc.robot.commands.endgamecommands.EndgameManagerCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.*;
import frc.robot.commands.intakecommands.intakemotorcommands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;
import frc.robot.utilities.SimulatedDrivetrain;
import frc.robot.utilities.PathPlannerSequentialCommandGroupUtility;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.EndgameSensorUtility;
import frc.robot.utilities.DriveCameraUtility.BallColor;

public class RobotContainer {

    // ----- XBOX CONTROLLER(S) -----\\

    // Driver Controller
    private final ControllerManager driverController = new ControllerManager(0);
    // Codriver Controllerd
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
    // ----- CAMERA -----\\

    // Camera subsystem for reflective tape
    private final VisionCameraSubsystem reflectiveTapeCameraSubsystem;
    // Camera subsystem for cargo balls
    private final VisionCameraSubsystem cargoCameraSubsystem;

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

    // ----- CATAPULT -----\\

    // Indexer subsystem
    private final IndexerMotorSubsystem indexerMotorSubsystem;

    // ----- ENDGAME -----\\
    private final EndgameManagerCommand endgameManager;
    // Endgame Arm Commands
    private final EndgameMotorSubsystem endgameMotorSubsystem;
    private final EndgameArmCommand endgameArmCommand;
    private final EndgameArmRevCommand endgameArmRevCommand;

    // Endgame Piston Subsystems
    private final EndgamePistonSubsystem endgamePiston1;
    private final EndgamePistonSubsystem endgamePiston2;
    private final EndgamePistonSubsystem endgamePistonL3;
    private final EndgamePistonSubsystem endgamePistonR3;
    private final EndgamePistonSubsystem endgamePiston4;

    private final HubAimingCommand hubAimingCommand;

    // LED commands
    private final LEDCommand autonPatternCommand;
    private final LEDCommand idlePatternCommand;
    private final LEDCommand endgamePatternCommand;

    private final LEDSubsystem ledSubsystem;
    // ----- AUTONOMOUS -----\\
    private final AutoCommandManager autoManager;

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

        // ----- CAMERA SUBSYSTEM INITS -----\\
        // Camera subsystem for reflective tape
        reflectiveTapeCameraSubsystem = new VisionCameraSubsystem(
                VisionCameraSubsystem.CameraType.REFLECTIVE_TAPE);
        // Camera subsystem for cargo balls
        cargoCameraSubsystem = new VisionCameraSubsystem(
                VisionCameraSubsystem.CameraType.BALL_DETECTOR);
        PortForwarder.add(5800, "10.9.30.25", 5800);
        PortForwarder.add(1181, "10.9.30.25", 1181);
        PortForwarder.add(1182, "10.9.30.25", 1182);
        PortForwarder.add(1183, "10.9.30.25", 1183);
        PortForwarder.add(1184, "10.9.30.25", 1184);

        // ----- INTAKE SUBSYSTEM INITS -----\\
        // Intake has to be instantiated before drive subsystem because we need to
        // initialize the gyro
        intakeMotorSubsystem = new IntakeMotorSubsystem(5, 9);
        intakePistonSubsystem = new IntakePistonSubsystem(1, 12);

        // ----- DRIVETRAIN SUBSYSTEM INITS -----\\

        shifterSubsystem = new ShifterSubsystem(0);
        driveSubsystem = new DriveSubsystem(1, 8, 2, 7);

        // ----- CATAPULT SUBSYSTEM INITS -----\\
        indexerMotorSubsystem = new IndexerMotorSubsystem(6);

        // ----- ENDGAME SUBSYSTEM INITS -----\\
        // Endgame Motor Subsystems
        endgameMotorSubsystem = new EndgameMotorSubsystem(3, 4);
        // Endgame Piston Subsystems
        endgamePiston1 = new EndgamePistonSubsystem(8);
        endgamePiston2 = new EndgamePistonSubsystem(9);
        endgamePistonL3 = new EndgamePistonSubsystem(10);
        endgamePistonR3 = new EndgamePistonSubsystem(14);
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
        autoManager.addSubsystem(subNames.VisionCameraSubsystem, reflectiveTapeCameraSubsystem);

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
                reflectiveTapeCameraSubsystem,
                cargoCameraSubsystem,
                driverController.getController());

        // ----- DRIVETRAIN SHIFTER COMMAND INITS -----\\
        toggleShifterCommand = new ToggleShifterCommand(shifterSubsystem);

        // ----- ENDGAME COMMAND INITS -----\\
        // Endgame Arm Commands
        endgameArmCommand = new EndgameArmCommand(endgameMotorSubsystem);
        endgameArmRevCommand = new EndgameArmRevCommand(endgameMotorSubsystem);
        endgameManager = new EndgameManagerCommand(endgameMotorSubsystem,
                endgamePiston1, endgamePiston2, endgamePistonL3, endgamePistonR3, endgamePiston4);

        hubAimingCommand = new HubAimingCommand(reflectiveTapeCameraSubsystem, driveSubsystem);

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
        scheduler.setDefaultCommand(endgamePiston1, new EndgameCloseClawSingleCommand(endgamePiston1));
        scheduler.setDefaultCommand(endgamePiston2, new EndgameCloseClawSingleCommand(endgamePiston2));
        scheduler.setDefaultCommand(endgamePistonL3, new EndgameCloseClawSingleCommand(endgamePistonL3));
        scheduler.setDefaultCommand(endgamePistonR3, new EndgameCloseClawSingleCommand(endgamePistonR3));
        scheduler.setDefaultCommand(endgamePiston4, new EndgameCloseClawSingleCommand(endgamePiston4));
        scheduler.setDefaultCommand(indexerMotorSubsystem, new IndexerForwardCommand(indexerMotorSubsystem, false));
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

        driverController.getRightBumper().whileActiveContinuous(hubAimingCommand);
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
        resheduleAutononmousLEDS(false);
    }

    public void startCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        if (camera != null) {
            camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            camera.setFPS(CAMERA_FPS);
        }
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
        resheduleAutononmousLEDS(true);

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
                endgamePistonL3.open();
            } else {
                endgamePistonL3.closed();
            }
            if (driverController.getXButton().get()) {
                endgamePistonR3.open();
            } else {
                endgamePistonR3.closed();
            }
            if (driverController.getRightBumper().get()) {
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
        resheduleAutononmousLEDS(true);
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

    private void resheduleAutononmousLEDS(boolean useAutonomousLEDCmd) {
        LEDCommand ledCommand = (useAutonomousLEDCmd) ? autonPatternCommand : idlePatternCommand;
        LEDCommand endledCommand = (useAutonomousLEDCmd) ? idlePatternCommand : autonPatternCommand;
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.unregisterSubsystem(ledSubsystem);
        endledCommand.cancel();
        scheduler.setDefaultCommand(ledSubsystem, ledCommand);
    }

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
    }
} // End of RobotContainer