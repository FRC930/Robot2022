package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.utilities.ShifterUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>ToggleShifterCommand</h3>
 * 
 * Toggles the shifter to change what gear ratio we are using.
 */
public class ToggleShifterCommand extends CommandBase {
    private ShifterSubsystem shifterSubsystem;

    /**
     * <h3>ToggleShifterCommand</h3>
     * 
     * <p>
     * Initializes a {@link frc.robot.commands.ToggleShifterCommand
     * ToggleShifterCommand} with the passed subsystems
     * </p>
     * 
     * @param sSubsystem a ShifterSubsystem representing the solenoid
     * @param dSubsystem a DriveSubsystem representing the drive base of the robot
     */
    public ToggleShifterCommand(ShifterSubsystem sSubsystem) {
        shifterSubsystem = sSubsystem;

        shifterSubsystem.setShifterState(false);

        addRequirements(shifterSubsystem);
    }

    @Override
    public void initialize() {
        shifterSubsystem.setShifterState(true);
    }

    @Override
    public void end(boolean interrupted) {
        shifterSubsystem.setShifterState(false);
    }
}
