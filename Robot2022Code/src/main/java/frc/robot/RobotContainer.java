package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CatapultCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.commands.endgamecommands.EndgameArmCommand;
import frc.robot.commands.endgamecommands.EndgameArmRevCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.ClockwiseIntakeMotorsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.CounterclockwiseIntakeMotorsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.StopIntakeMotorsCommand;
import frc.robot.subsystems.CatapultSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndgameMotorSubsystem;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.subsystems.EndgameSensorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.triggers.AxisTrigger;

public class RobotContainer {
    public static final int XB_AXIS_LEFT_X = 0;
    public static final int XB_AXIS_LEFT_Y = 1;
    public static final int XB_AXIS_RIGHT_X = 4;
    public static final int XB_AXIS_RIGHT_Y = 5;
    public static final int XB_AXIS_LT = 2;
    public static final int XB_AXIS_RT = 3;

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

    // The driver controller
    private final XboxController controller = new XboxController(0);

    private final DriveSubsystem driveSubsystem;
    private final DriveCommand driveCommand;

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

    private final CatapultSubsystem catapultSubsystem;
    private final CatapultCommand catapultCommand;

    private final ShifterSubsystem shifterSubsystem;
    private final ToggleShifterCommand toggleShifterCommand;

    private final IntakeMotorSubsystem intakeMotorSubsystem;
    private final ClockwiseIntakeMotorsCommand clockwiseIntakeMotorsCommand;
    private final CounterclockwiseIntakeMotorsCommand counterClockwiseIntakeMotorsCommand;
    private final StopIntakeMotorsCommand stopIntakeMotorsCommand;


    /**
     * <h3>RobotContainer</h3>
     * 
     * Initializes the robot
     */
    public RobotContainer() {
        endgameMotorSubsystem = new EndgameMotorSubsystem(3, 4);
        endgameArmCommand = new EndgameArmCommand(endgameMotorSubsystem);
        endgameArmRevCommand = new EndgameArmRevCommand(endgameMotorSubsystem);

        left2Sensor = new EndgameSensorSubsystem(1);
        right2Sensor = new EndgameSensorSubsystem(2);
        left4Sensor = new EndgameSensorSubsystem(3);
        right4Sensor = new EndgameSensorSubsystem(4);

        left1piston = new EndgamePistonSubsystem(3);
        left2piston = new EndgamePistonSubsystem(4);
        left3piston = new EndgamePistonSubsystem(5);
        left4piston = new EndgamePistonSubsystem(6);
        right1piston = new EndgamePistonSubsystem(7);
        right2piston = new EndgamePistonSubsystem(8);
        right3piston = new EndgamePistonSubsystem(9);
        right4piston = new EndgamePistonSubsystem(10);

        driveSubsystem = new DriveSubsystem(1, 2);
        driveCommand = new DriveCommand(driveSubsystem, endgameMotorSubsystem, controller);

        catapultSubsystem = new CatapultSubsystem(1, 2);
        catapultCommand = new CatapultCommand(catapultSubsystem);

        shifterSubsystem = new ShifterSubsystem(0);
        toggleShifterCommand = new ToggleShifterCommand(shifterSubsystem, driveSubsystem);

        intakeMotorSubsystem = new IntakeMotorSubsystem(5);
        clockwiseIntakeMotorsCommand = new ClockwiseIntakeMotorsCommand(intakeMotorSubsystem);
        counterClockwiseIntakeMotorsCommand = new CounterclockwiseIntakeMotorsCommand(intakeMotorSubsystem);
        stopIntakeMotorsCommand = new StopIntakeMotorsCommand(intakeMotorSubsystem);
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
        AxisTrigger shifterTrigger = new AxisTrigger(controller, XB_AXIS_RT);
        shifterTrigger.whileActiveOnce(toggleShifterCommand);

        JoystickButton launchButton = new JoystickButton(controller, XB_RB);
        launchButton.whileActiveOnce(catapultCommand);

        AxisTrigger reverseIntakeButton = new AxisTrigger(controller, XB_AXIS_LT);
        reverseIntakeButton.whileActiveOnce(clockwiseIntakeMotorsCommand);
        
        JoystickButton intakeButton = new JoystickButton(controller, XB_LB);
        intakeButton.whileActiveOnce(counterClockwiseIntakeMotorsCommand);

        JoystickButton stopIntake = new JoystickButton(controller, XB_LB);
        stopIntake.whenReleased(stopIntakeMotorsCommand);

        AxisTrigger stopReverseIntake = new AxisTrigger(controller, XB_AXIS_LT);
        stopReverseIntake.whenInactive(stopIntakeMotorsCommand);

        JoystickButton rotateArmButton = new JoystickButton(controller, XB_Y);
        rotateArmButton.whileActiveOnce(endgameArmCommand);

        JoystickButton rotateArmRevButton = new JoystickButton(controller, XB_A);
        rotateArmRevButton.whileActiveOnce(endgameArmRevCommand);

        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.unregisterSubsystem(driveSubsystem);

        scheduler.setDefaultCommand(driveSubsystem, driveCommand);
        
    }
}
