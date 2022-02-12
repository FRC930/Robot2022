package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import frc.robot.ControllerManager;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexerForwardCommand;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.commands.autocommands.AutoCommandManager;
import frc.robot.commands.autocommands.AutoCommandManager.subNames;
import frc.robot.commands.autocommands.paths.BottomBackShootCommand;
import frc.robot.commands.autocommands.paths.BottomBackSideShootCommand;
import frc.robot.commands.autocommands.paths.DefaultAutoPathCommand;
import frc.robot.commands.endgamecommands.EndgameArmCommand;
import frc.robot.commands.endgamecommands.EndgameArmRevCommand;
import frc.robot.commands.endgamecommands.EndgameCloseClawPairCommand;
import frc.robot.commands.endgamecommands.EndgameCloseClawSingleCommand;
import frc.robot.commands.endgamecommands.EndgameCloseWhenAway;
import frc.robot.commands.endgamecommands.EndgameCloseWhenTouching;
import frc.robot.commands.endgamecommands.EndgameCommandManager;
import frc.robot.commands.endgamecommands.EndgameOpenClawPairCommand;
import frc.robot.commands.endgamecommands.EndgameOpenClawSingleCommand;
import frc.robot.commands.endgamecommands.EndgameRotateHorizonalCommand;
import frc.robot.commands.endgamecommands.EndgameRotateVerticalCommand;
import frc.robot.commands.intakecommands.intakePistonCommands.*;
import frc.robot.commands.intakecommands.intakemotorcommands.*;
import frc.robot.subsystems.BallSensorSubsystem;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;
import frc.robot.triggers.AxisTrigger;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.SimulatedDrivetrain;
import frc.robot.utilities.SequentialCommandGroupWithTraj;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.DriveCameraUtility.BallColor;


public class RobotContainer {

    // ----- XBOX CONTROLLER(S) -----\\

    // Driver Controller
    private final ControllerManager driverController = new ControllerManager(0);
    // Codriver Controllerd
    private final ControllerManager codriverController = new ControllerManager(1);
/*
    // Left Joystick
    public static final int XB_AXIS_LEFT_X = 0;
    public static final int XB_AXIS_LEFT_Y = 1;
    // Triggers
    public static final int XB_AXIS_LT = 2;
    public static final int XB_AXIS_RT = 3;
    // Right Joystick
    public static final int XB_AXIS_RIGHT_X = 4;
    public static final int XB_AXIS_RIGHT_Y = 5;

    // Buttons
    public static final int XB_A = 1;
    public static final int XB_B = 2;
    public static final int XB_X = 3;
    public static final int XB_Y = 4;
    public static final int XB_LB = 5;
    public static final int XB_RB = 6;
    public static final int XB_BACK = 7;
    public static final int XB_START = 8;
    public static final int XB_LEFTSTICK_BUTTON = 9;
    public static final int XB_RIGHTSTICK_BUTTON = 10;
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

    // Catapult Subsystem
    private final CatapultSubsystem catapultSubsystem;
    private final IndexerMotorSubsystem indexerMotorSubsystem;
    private final BallSensorSubsystem catapultSensor;
    private final BallSensorSubsystem indexerSensor;
    // Catapult Launch Command
    private final CatapultCommand catapultCommand;

    // ----- ENDGAME -----\\
    private final EndgameCommandManager endgameManager;
    // Endgame Arm Commands
    private final EndgameMotorSubsystem endgameMotorSubsystem;
    private final EndgameArmCommand endgameArmCommand;
    private final EndgameArmRevCommand endgameArmRevCommand;

    // Endgame Piston Subsystems
    private final EndgamePistonSubsystem left1piston;
    private final EndgamePistonSubsystem left2piston;
    private final EndgamePistonSubsystem left3piston;
    private final EndgamePistonSubsystem left4piston;
    private final EndgamePistonSubsystem right1piston;
    private final EndgamePistonSubsystem right2piston;
    private final EndgamePistonSubsystem right3piston;
    private final EndgamePistonSubsystem right4piston;

    // ----- AUTONOMOUS -----\\

    private final AutoCommandManager autoManager;
    // Default

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
         * CONSTRUCT COMMAND MANAGER
         * -----------------------------------------------------------------------------
         * ---
         */
        autoManager = new AutoCommandManager();
        
        /*
         * -----------------------------------------------------------------------------
         * ---
         * SUBSYSTEM INITIALIZATIONS
         * -----------------------------------------------------------------------------
         * ---
         */

        // ----- CAMERA SUBSYSTEM INITS -----\\

        // Camera subsystem for reflective tape
        reflectiveTapeCameraSubsystem = new VisionCameraSubsystem(
                VisionCameraSubsystem.CameraType.REFLECTIVE_TAPE);
        // Camera subsystem for cargo balls
        cargoCameraSubsystem = new VisionCameraSubsystem(
                VisionCameraSubsystem.CameraType.BALL_DETECTOR);

        PortForwarder.add(5800, "10.9.30.25", 5800);

        // ----- INTAKE SUBSYSTEM INITS -----\\

        // Intake has to be instantiated before drive subsystem because we need to
        // initialize the gyro
        intakeMotorSubsystem = new IntakeMotorSubsystem(5);
        intakePistonSubsystem = new IntakePistonSubsystem(1);

        // ----- DRIVETRAIN SUBSYSTEM INITS -----\\

        driveSubsystem = new DriveSubsystem(1, 8, 2, 7);
        autoManager.addSubsystem(subNames.DriveSubsystem, driveSubsystem);

        // ----- DRIVETRAIN SHIFTER SUBSYSTEM INITS -----\\

        shifterSubsystem = new ShifterSubsystem(0);

        // ----- CATAPULT SUBSYSTEM INITS -----\\

        // TODO:ADD CATAPULT SENSOR
        // TODO:ADD SOLENOID ID 7 FOR HARD-STOP
        catapultSubsystem = new CatapultSubsystem(2, 3, 4, 5, 6);
        indexerMotorSubsystem = new IndexerMotorSubsystem(6);
        catapultSensor = new BallSensorSubsystem(0);
        indexerSensor = new BallSensorSubsystem(5);
        // ----- CATAPULT COMMAND INITS -----\\

        catapultCommand = new CatapultCommand(catapultSubsystem);

        // ----- ENDGAME SUBSYSTEM INITS -----\\

        // Endgame Motor Subsystems
        endgameMotorSubsystem = new EndgameMotorSubsystem(3, 4);

        // Endgame Piston Subsystems
        left1piston = new EndgamePistonSubsystem(8);
        left2piston = new EndgamePistonSubsystem(9);
        left3piston = new EndgamePistonSubsystem(10);
        left4piston = new EndgamePistonSubsystem(11);
        right1piston = new EndgamePistonSubsystem(12);
        right2piston = new EndgamePistonSubsystem(13);
        right3piston = new EndgamePistonSubsystem(14);
        right4piston = new EndgamePistonSubsystem(15);

        endgameManager = new EndgameCommandManager(endgameMotorSubsystem, 
        left1piston, left2piston, left3piston, left4piston, right1piston, 
        right2piston, right3piston, right4piston);
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
        runIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeMotorSubsystem, indexerSensor, false);
        reverseIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeMotorSubsystem, indexerSensor, true);
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
    
        // ----- SETTING BALL COLOR -----\\

        if (DriverStation.getAlliance() == Alliance.Blue) {
            DriveCameraUtility.getInstance().setBallColor(BallColor.BLUE);
        } else {
            DriveCameraUtility.getInstance().setBallColor(BallColor.RED);
        }

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

        // DRIVER CONTROLLER BINDS
        /*
        AxisTrigger shifterTrigger = new AxisTrigger(driverController, XB_AXIS_RT);

        JoystickButton launchButton = new JoystickButton(driverController, XB_LB);
        JoystickButton rotateArmButton = new JoystickButton(driverController, XB_Y);
        JoystickButton rotateArmRevButton = new JoystickButton(driverController, XB_A);
        JoystickButton endgameSensorCloseButton = new JoystickButton(driverController, XB_X);
        JoystickButton rotateUntilTouchingButton = new JoystickButton(driverController, XB_B);
        JoystickButton endgameComplete = new JoystickButton(driverController, XB_START);
        JoystickButton indexerButton = new JoystickButton(codriverController, XB_RB);

        // CODRIVER CONTROLLER BINDS
        JoystickButton intakeButton = new JoystickButton(codriverController, XB_LB);
        JoystickButton reverseIntakeButton = new JoystickButton(codriverController, XB_B);
*/
        // Shifts the drivetrain when shifter trigger is pulled
        driverController.getRightTrigger().whileActiveOnce(toggleShifterCommand);

        // Launches a cargo ball when the launch button is pressed
        driverController.getLeftBumper().whileActiveOnce(catapultCommand);

        // Checks if LB is pressed, then it will engage the intake pistons
        codriverController.getLeftBumper().whileActiveOnce(engageIntakePistonsCommand);

        // Checks if LB is pressed and B isn't pressed, then it will run intake
        codriverController.getLeftBumper().and(codriverController.getBButton().negate()).whileActiveOnce(runIntakeMotorsCommand);
        // Checks if LB and B is pressed, then it will reverse the intake
        codriverController.getLeftBumper().and(codriverController.getBButton()).whileActiveOnce(reverseIntakeMotorsCommand);

        codriverController.getRightBumper().whileActiveOnce(new IndexerForwardCommand(indexerMotorSubsystem));

        // Manually rotates the endgame arms while pressed
        driverController.getYButton().whileActiveOnce(endgameArmCommand);

        // Manually rotates the endgame arms in reverse while pressed
        driverController.getAButton().whileActiveOnce(endgameArmRevCommand);

        /*
        driverController.getStartButton().whileActiveOnce(new SequentialCommandGroup(
            */
                // TODO:USE ENCODER AS PROGRESS TOOL
                /* verticalCommand-NEED TO GET ENCODER WORKING!!!, */
                
                /*
                endgame2, endgame3, endgame4, endgame5, endgame6, new WaitCommand(ENDGAME_RELEASE_DELAY), endgame8,
                endgame9, endgame10
                */
        /* new verticalCommand-NEED TO GET ENDCODER WORKING!! */
        //));

        // startCamera();

        // Manages commands via stacking
        CommandScheduler scheduler = CommandScheduler.getInstance();

        /*
         * Unregisters subsystems to prevent hanging resources
         */
        scheduler.unregisterSubsystem(
                driveSubsystem, // Drivetrain
                intakeMotorSubsystem, intakePistonSubsystem, // Intake
                left1piston, left2piston, left3piston, left4piston, // Endgame Left Arm
                right1piston, right2piston, right3piston, right4piston, // Endgame Right Arm
                /* , endgameMotorSubsystem */ // Endgame Motors
                indexerMotorSubsystem);

        // DRIVETRAIN DEFAULTS
        scheduler.setDefaultCommand(driveSubsystem, driveCommand);

        // INTAKE DEFAULTS
        scheduler.setDefaultCommand(intakeMotorSubsystem, stopIntakeMotorsCommand);
        scheduler.setDefaultCommand(intakePistonSubsystem, disengageIntakePistonsCommand);

        // ENDGAME DEFAULTS
        // scheduler.setDefaultCommand(endgameMotorSubsystem, new
        // EndgameRotateHorizonalCommand(endgameMotorSubsystem)); // -GET ENCODER
        // WORKING
        scheduler.setDefaultCommand(left1piston, new EndgameCloseClawSingleCommand(left1piston));
        scheduler.setDefaultCommand(left2piston, new EndgameCloseClawSingleCommand(left2piston));
        scheduler.setDefaultCommand(left3piston, new EndgameCloseClawSingleCommand(left3piston));
        scheduler.setDefaultCommand(left4piston, new EndgameCloseClawSingleCommand(left4piston));
        scheduler.setDefaultCommand(right1piston, new EndgameCloseClawSingleCommand(right1piston));
        scheduler.setDefaultCommand(right2piston, new EndgameCloseClawSingleCommand(right2piston));
        scheduler.setDefaultCommand(right3piston, new EndgameCloseClawSingleCommand(right3piston));
        scheduler.setDefaultCommand(right4piston, new EndgameCloseClawSingleCommand(right4piston));
        //scheduler.setDefaultCommand(indexerMotorSubsystem, new IndexerForwardCommand(indexerMotorSubsystem));
    }

    private void startCamera() {
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

        // --The instance of the scheduler
        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.unregisterSubsystem(catapultSubsystem,
                // catapultSensorSubsystem,
                driveSubsystem,
                endgameMotorSubsystem,
                // endgamePistonSubsystem,
                // endgameSensorSubsystem,
                intakeMotorSubsystem,
                // intakePistonSubsystem,
                shifterSubsystem// ,
        // visionCameraSubsystem
        );
        // TODO set default command for each subsystem
        // scheduler.setDefaultCommand(driveSubsystem, driveCommand);
    }

    public void testInit() {
        stopSubsystems();
    }

    public void testPeriodic() {
        if (driverController.getYButton().get()) {
            endgameMotorSubsystem.setMotorSpeed(0.2);
        } else if (driverController.getAButton().get()) {
            endgameMotorSubsystem.setMotorSpeed(-0.2);
        } else {
            endgameMotorSubsystem.setMotorSpeed(0.0);
        }
    }

    public void stopSubsystems() {
        endgameMotorSubsystem.setMotorSpeed(0.0);
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
        m_autocmd = autoManager.getAutonomousCommand();
        if (m_autocmd != null) {
            if (m_autocmd instanceof SequentialCommandGroupWithTraj) {
                List<Trajectory> list = ((SequentialCommandGroupWithTraj) m_autocmd).getTrajectories();
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
}
