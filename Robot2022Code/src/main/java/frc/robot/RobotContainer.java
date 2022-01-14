package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ToggleShifterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShifterSubsystem;

public class RobotContainer {
    // The driver controller
    private final XboxController controller = new XboxController(0);

    private final DriveSubsystem driveSubsystem;
    private final DriveCommand driveCommand;
    private final ShifterSubsystem shifterSubsystem;
    private final ToggleShifterCommand toggleShifterCommand;

    /**
     * <h3>RobotContainer</h3>
     * 
     * Initializes the robot
     */
    public RobotContainer() {
        driveSubsystem = new DriveSubsystem();
        driveCommand = new DriveCommand(driveSubsystem, controller);
        shifterSubsystem = new ShifterSubsystem(ShifterSubsystem.shifterSolenoidID);
        toggleShifterCommand = new ToggleShifterCommand(shifterSubsystem, driveSubsystem);
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
        JoystickButton toggleShifterButton = new JoystickButton(controller, 1);
        toggleShifterButton.whenPressed(toggleShifterCommand);

        CommandScheduler scheduler = CommandScheduler.getInstance();

        scheduler.unregisterSubsystem(driveSubsystem);

        scheduler.setDefaultCommand(driveSubsystem, driveCommand);
    }
}
