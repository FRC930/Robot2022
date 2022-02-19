//----- IMPORTS -----\\

package frc.robot.commands.intakecommands.intakePistonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePistonSubsystem;

//----- CLASS -----\\
/**
 * <h3>Engage Intake Piston Command</h3>
 * 
 * Engages the intake pistons.
 */
public class DisengageIntakePistonsCommand extends CommandBase{
    
    //----- SUBSYSTEM(S) -----\\

    private IntakePistonSubsystem intakePistonSubsystem;

    //----- CONSTRUCTOR -----\\
    /**
     * <h3>Engage Piston Intake Piston Command</h3>
     * 
     * Engages the intake piston.
     * 
     * @param ipSubsystem Intake Piston Subsystem
     */
    public DisengageIntakePistonsCommand(IntakePistonSubsystem ipSubsystem) {
        intakePistonSubsystem = ipSubsystem;
        addRequirements(intakePistonSubsystem);
    }

    /**
     * <h3>initialize</h3>
     * 
     * Called when the command is initialized.
     */
    @Override
    public void initialize() {
        intakePistonSubsystem.setIntakePistonState(false);
    }

    /**
     * <h3>isFinished</h3>
     * 
     * Called when the command ends.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

}
