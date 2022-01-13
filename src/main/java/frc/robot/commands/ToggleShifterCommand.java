package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShifterSubsystem;

/**
 * <h3>ToggleShifterCommand</h3>
 * 
 * Toggles the shifter to change what gear ratio we are using.
 */
public class ToggleShifterCommand extends CommandBase {
    private ShifterSubsystem shifterSubsystem;
    private DriveSubsystem driveSubsystem;

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
    public ToggleShifterCommand(ShifterSubsystem sSubsystem, DriveSubsystem dSubsystem) {
        shifterSubsystem = sSubsystem;
        driveSubsystem = dSubsystem;
    }

    @Override
    public void initialize() {
        shifterSubsystem.setShifterState(!shifterSubsystem.getShifterState());
        driveSubsystem.setPistonState(shifterSubsystem.getShifterState());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
