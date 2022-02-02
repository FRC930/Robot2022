package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.commands.autocommands.paths.BottomBackShootCommand;
import frc.robot.commands.autocommands.paths.BottomBackSideShootCommand;
import frc.robot.commands.autocommands.paths.DefaultAutoPathCommand;
import frc.robot.commands.endgamecommands.EndgameArmCommand;
import frc.robot.commands.endgamecommands.EndgameArmRevCommand;
import frc.robot.commands.endgamecommands.EndgameCloseClawCommand;
import frc.robot.commands.endgamecommands.EndgameCloseWhenAway;
import frc.robot.commands.endgamecommands.EndgameCloseWhenTouching;
import frc.robot.commands.endgamecommands.EndgameOpenClawCommand;
import frc.robot.commands.endgamecommands.EndgameRotateVerticalCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.StopIntakeMotorsCommand;
import frc.robot.subsystems.CatapultSensorSubsystem;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.EndgameSensorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.VisionCameraSubsystem;
import frc.robot.triggers.AxisTrigger;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.DriveCameraUtility.BallColor;

public class RobotContainer {
    public static final int XB_AXIS_LEFT_X = 0;
    public static final int XB_AXIS_LEFT_Y = 1;
    public static final int XB_AXIS_LT = 2;
    public static final int XB_AXIS_RT = 3;
    public static final int XB_AXIS_RIGHT_X = 4;
    public static final int XB_AXIS_RIGHT_Y = 5;

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

    public static final int CAMERA_WIDTH = 160;
    public static final int CAMERA_HEIGHT = 120;
    public static final int CAMERA_FPS = 30;

    // The driver controller
    private final XboxController driverController = new XboxController(0);
    private final XboxController codriverController = new XboxController(1);

    private final DriveSubsystem driveSubsystem;
    private final DriveCommand driveCommand;

    private final EndgameCloseWhenTouching endgameCloseTouchingLeft3;
    private final EndgameCloseWhenTouching endgameCloseTouchingRight3;
    private final EndgameCloseWhenTouching endgameCloseTouchingLeft1;
    private final EndgameCloseWhenTouching endgameCloseTouchingRight1;

    private final EndgameMotorSubsystem endgameMotorSubsystem;
    private final EndgameArmCommand endgameArmCommand;
    private final EndgameArmRevCommand endgameArmRevCommand;

    private final EndgameSensorSubsystem left2Sensor;
    private final EndgameSensorSubsystem right2Sensor;
    private final EndgameSensorSubsystem left4Sensor;
    private final EndgameSensorSubsystem right4Sensor;

    private final EndgamePistonSubsystem left1piston;
    private final EndgamePistonSubsystem left2piston;
    private final EndgamePistonSubsystem left3piston;
    private final EndgamePistonSubsystem left4piston;
    private final EndgamePistonSubsystem right1piston;
    private final EndgamePistonSubsystem right2piston;
    private final EndgamePistonSubsystem right3piston;
    private final EndgamePistonSubsystem right4piston;

    private final double ENDGAME_PISTON_DELAY = 0.25;
    private final double ENDGAME_RELEASE_DELAY = 5;

    private final EndgameRotateVerticalCommand verticalCommand;
    private final ParallelRaceGroup endgame2;
    private final ParallelCommandGroup endgame3;
    private final ParallelRaceGroup endgame4;
    private final ParallelRaceGroup endgame5;
    private final ParallelRaceGroup endgame6;
    private final ParallelRaceGroup endgame8;
    private final ParallelRaceGroup endgame9;
    private final ParallelRaceGroup endgame10;

    private final CatapultSubsystem catapultSubsystem;
    private final CatapultCommand catapultCommand;

    private final ShifterSubsystem shifterSubsystem;
    private final ToggleShifterCommand toggleShifterCommand;

    private final IntakeMotorSubsystem intakeMotorSubsystem;
    //private final IntakePistonSubsystem intakePistonSubsystem;
    private final RunIntakeMotorsCommand runIntakeMotorsCommand;
    private final RunIntakeMotorsCommand reverseIntakeMotorsCommand;
    private final StopIntakeMotorsCommand stopIntakeMotorsCommand;

    private final DefaultAutoPathCommand defaultAutoPathCommand;
    private final BottomBackShootCommand bottomBackShootCommand;
    private final BottomBackSideShootCommand bottomBackSideShootCommand;

    /**
     * <h3>RobotContainer</h3>
     * 
     * Initializes the robot
     */
    public RobotContainer() {
        endgameMotorSubsystem = new EndgameMotorSubsystem(3, 4);

        endgameArmCommand = new EndgameArmCommand(endgameMotorSubsystem);
        endgameArmRevCommand = new EndgameArmRevCommand(endgameMotorSubsystem);
        verticalCommand = new EndgameRotateVerticalCommand(endgameMotorSubsystem);

        left2Sensor = new EndgameSensorSubsystem(1);
        right2Sensor = new EndgameSensorSubsystem(2);
        left4Sensor = new EndgameSensorSubsystem(3);
        right4Sensor = new EndgameSensorSubsystem(4);

        left1piston = new EndgamePistonSubsystem(8);
        left2piston = new EndgamePistonSubsystem(9);
        left3piston = new EndgamePistonSubsystem(10);
        left4piston = new EndgamePistonSubsystem(11);
        right1piston = new EndgamePistonSubsystem(12);
        right2piston = new EndgamePistonSubsystem(13);
        right3piston = new EndgamePistonSubsystem(14);
        right4piston = new EndgamePistonSubsystem(15);

        endgameCloseTouchingLeft3 = new EndgameCloseWhenTouching(left3piston, left4Sensor);
        endgameCloseTouchingRight3 = new EndgameCloseWhenTouching(right3piston, right4Sensor);
        endgameCloseTouchingLeft1 = new EndgameCloseWhenTouching(left1piston, left2Sensor);
        endgameCloseTouchingRight1 = new EndgameCloseWhenTouching(right1piston, right2Sensor);

        //Opens #3 claws
        endgame2 = new ParallelRaceGroup(new EndgameOpenClawCommand(left3piston),
                new EndgameOpenClawCommand(right3piston), new WaitCommand(ENDGAME_PISTON_DELAY));
        //Closes #3 claws independently on both sides when sensors trigger
        endgame3 = new ParallelCommandGroup(new EndgameCloseWhenTouching(left3piston, left4Sensor),
                new EndgameCloseWhenTouching(right3piston, right4Sensor));
        //Opens #1 claws
        endgame4 = new ParallelRaceGroup(new EndgameOpenClawCommand(left1piston),
                new EndgameOpenClawCommand(right1piston), new WaitCommand(ENDGAME_PISTON_DELAY));
        //Rotates arm until one #2 sensor triggers, closing all arms and stops motor
        endgame5 = new ParallelRaceGroup(new EndgameArmCommand(endgameMotorSubsystem),
                new EndgameCloseWhenTouching(left1piston, left2Sensor),
                new EndgameCloseWhenTouching(right1piston, right2Sensor));
        //Opens #3 and #4 claws
        endgame6 = new ParallelRaceGroup(new EndgameOpenClawCommand(left3piston),
                new EndgameOpenClawCommand(right3piston), new EndgameOpenClawCommand(left4piston),
                new EndgameOpenClawCommand(right4piston), new WaitCommand(ENDGAME_PISTON_DELAY));
        //Closes #4 claws
        endgame8 = new ParallelRaceGroup(new EndgameCloseClawCommand(left4piston),
                new EndgameCloseClawCommand(right4piston), new WaitCommand(ENDGAME_PISTON_DELAY));
        //Rotates arm until one #4 sensor triggers, closing all arms and stops motor
        endgame9 = new ParallelRaceGroup(new EndgameArmCommand(endgameMotorSubsystem),
                new EndgameCloseWhenTouching(left3piston, left4Sensor),
                new EndgameCloseWhenTouching(right3piston, right4Sensor));
        //Opens #2 claws-NOTE:Claws closed after delay ends due to default command
        endgame10 = new ParallelRaceGroup(new EndgameOpenClawCommand(left2piston),
                new EndgameOpenClawCommand(right2piston), new WaitCommand(ENDGAME_RELEASE_DELAY));

        VisionCameraSubsystem reflectiveTapeSubsystem = new VisionCameraSubsystem(
                VisionCameraSubsystem.CameraType.REFLECTIVE_TAPE);
        VisionCameraSubsystem ballSubsystem = new VisionCameraSubsystem(VisionCameraSubsystem.CameraType.BALL_DETECTOR);

        // Intake has to be instantiated before drive subsystem because we need to
        // initialize the gyro
        intakeMotorSubsystem = new IntakeMotorSubsystem(5);
        //intakePistonSubsystem = new IntakePistonSubsystem(1);

        driveSubsystem = new DriveSubsystem(1, 2);
        driveCommand = new DriveCommand(driveSubsystem, endgameMotorSubsystem, reflectiveTapeSubsystem, ballSubsystem,
                driverController);

        //TODO:ADD CATAPULT SENSOR
        //TODO:ADD SOLENOID ID 7 FOR HARD-STOP
        catapultSubsystem = new CatapultSubsystem(2, 3, 4, 5, 6);
        catapultCommand = new CatapultCommand(catapultSubsystem);

        shifterSubsystem = new ShifterSubsystem(0);
        toggleShifterCommand = new ToggleShifterCommand(shifterSubsystem);

        // This is where the Autonomous commands are being added to shuffleboard 
        ShuffleboardUtility.getInstance().setDefaultAutonOptions("Default (None)", null);
        defaultAutoPathCommand = new DefaultAutoPathCommand(driveSubsystem);
        ShuffleboardUtility.getInstance().addAutonOptions("defaultAutoPathCommand", defaultAutoPathCommand);
        bottomBackShootCommand = new BottomBackShootCommand(driveSubsystem);
        ShuffleboardUtility.getInstance().addAutonOptions("bottomBackShootCommand", bottomBackShootCommand);
        bottomBackSideShootCommand = new BottomBackSideShootCommand(driveSubsystem);
        ShuffleboardUtility.getInstance().addAutonOptions("bottomBackSideShootCommand", bottomBackSideShootCommand);

        runIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeMotorSubsystem, false);
        reverseIntakeMotorsCommand = new RunIntakeMotorsCommand(intakeMotorSubsystem, true);
        stopIntakeMotorsCommand = new StopIntakeMotorsCommand(intakeMotorSubsystem);

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
        AxisTrigger shifterTrigger = new AxisTrigger(driverController, XB_AXIS_RT);
        shifterTrigger.whileActiveOnce(toggleShifterCommand);

        JoystickButton launchButton = new JoystickButton(driverController, XB_LB);
        launchButton.whileActiveOnce(catapultCommand);

        // Checks if LB and B is pressed, then it will reverse the intake
        new JoystickButton(codriverController, XB_LB).
            and(new JoystickButton(codriverController, XB_B)).
            whileActiveOnce(reverseIntakeMotorsCommand);
        // Checks if LB is pressed and B isn't pressed, then it will run intake
        new JoystickButton(codriverController, XB_LB).
            and(new JoystickButton(codriverController, XB_B).negate()).
            whileActiveOnce(runIntakeMotorsCommand);

        JoystickButton rotateArmButton = new JoystickButton(driverController, XB_Y);
        rotateArmButton.whileActiveOnce(endgameArmCommand);

        JoystickButton rotateArmRevButton = new JoystickButton(driverController, XB_A);
        rotateArmRevButton.whileActiveOnce(endgameArmRevCommand);

        // Two endgame commands used for testing
        JoystickButton endgameSensorCloseButton = new JoystickButton(driverController, XB_X);
        endgameSensorCloseButton.whileActiveOnce(new EndgameCloseWhenTouching(left1piston, left2Sensor));
        JoystickButton rotateUntilTouchingButton = new JoystickButton(driverController, XB_B);
        rotateUntilTouchingButton.whileActiveOnce(new SequentialCommandGroup(
                new ParallelRaceGroup(new EndgameArmCommand(endgameMotorSubsystem), 
                new EndgameCloseWhenTouching(left1piston, left2Sensor), endgameCloseTouchingRight1),
                new WaitCommand(10)));

        JoystickButton endgameComplete = new JoystickButton(driverController, XB_START);
        endgameComplete.whileActiveOnce(new SequentialCommandGroup(
                // TODO:USE ENCODER AS PROGRESS TOOL
                /* verticalCommand-NEED TO GET ENCODER WORKING!!!, */
                endgame2, endgame3, endgame4, endgame5, endgame6, new WaitCommand(ENDGAME_RELEASE_DELAY), endgame8,
                endgame9, endgame10
        /* new verticalCommand-NEED TO GET ENDCODER WORKING!! */
        ));

        // startCamera();

        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.unregisterSubsystem(left1piston, left2piston, left3piston, left4piston,
                right1piston, right2piston, right3piston, right4piston, driveSubsystem/* , endgameMotorSubsystem */);
        // scheduler.setDefaultCommand(endgameMotorSubsystem, new
        // EndgameRotateHorizonalCommand(endgameMotorSubsystem));-GET ENCODER WORKING
        scheduler.setDefaultCommand(left1piston, new EndgameCloseClawCommand(left1piston));
        scheduler.setDefaultCommand(left2piston, new EndgameCloseClawCommand(left2piston));
        scheduler.setDefaultCommand(left3piston, new EndgameCloseClawCommand(left3piston));
        scheduler.setDefaultCommand(left4piston, new EndgameCloseClawCommand(left4piston));
        scheduler.setDefaultCommand(right1piston, new EndgameCloseClawCommand(right1piston));
        scheduler.setDefaultCommand(right2piston, new EndgameCloseClawCommand(right2piston));
        scheduler.setDefaultCommand(right3piston, new EndgameCloseClawCommand(right3piston));
        scheduler.setDefaultCommand(right4piston, new EndgameCloseClawCommand(right4piston));
        
        scheduler.setDefaultCommand(intakeMotorSubsystem, stopIntakeMotorsCommand);

        scheduler.setDefaultCommand(driveSubsystem, driveCommand);
    }

    private void startCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        if (camera != null) {
            camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
            camera.setFPS(CAMERA_FPS);
        }
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

    public Command getAutonomousCommand() {
        return ShuffleboardUtility.getInstance().getSelectedAutonPath();
    }
}
