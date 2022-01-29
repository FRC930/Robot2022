//----- IMPORTS -----\\

package frc.robot.commands.intakecommands.intakePistonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePistonSubsystem;
import frc.robot.utilities.DriveCameraUtility;
import frc.robot.utilities.DriveCameraUtility.CameraStates;

//----- CLASS -----\\
/**
 * <h3>Engage Intake Piston Command</h3>
 * 
 * Engages the intake pistons.
 */
public class EngageIntakePistonsCommand extends CommandBase {

    // ----- SUBSYSTEM(S) -----\\

    private IntakePistonSubsystem intakePistonSubsystem;

    // ----- CONSTRUCTOR -----\\
    /**
     * <h3>Engage Piston Intake Piston Command</h3>
     * 
     * Engages the intake piston.
     * 
     * @param ipSubsystem Intake Piston Subsystem
     */
    public EngageIntakePistonsCommand(IntakePistonSubsystem ipSubsystem) {
        intakePistonSubsystem = ipSubsystem;
    }

    /**
     * <h3>initialize</h3>
     * 
     * Called when the command is initialized.
     */
    @Override
    public void initialize() {
        DriveCameraUtility.getInstance().setCameraState(CameraStates.BALL);
    }

    /**
     * <h3>execute</h3>
     * 
     * Called when the command is run by the scheduler.
     */
    @Override
    public void execute() {
        intakePistonSubsystem.setIntakePistonState(true);
    }

    /**
     * <h3>end</h3>
     * 
     * Called when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        DriveCameraUtility.getInstance().setCameraState(CameraStates.REFLECTIVE_TAPE);
    }

}
