//-------- IMPORTS --------\\

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CatapultSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h3>CatapultCommand</h3>
 * 
 * Launches the ball contained in the catapult
 * 
 * @author <a href="https://github.com/Jelombo">Jelombo</a>,
 *         <a href="https://github.com/awtpi314">awtpi314</a>
 * @since 01/20/2022
 * @version 1.0.0
 */
public class CatapultCommand extends CommandBase {

    //-------- VARIABLES --------\\

    CatapultSubsystem catapultSubsystem;
    CatapultPower power;

    //-------- CONSTRUCTOR --------\\
    /**
     * <h3>CatapultCommand</h3>
     * 
     * Initializes a new catapult command with the passed catapult subsystem
     *
     * @param catapult the {@link frc.robot.subsystems.CaptapultSubsystem
     *                 CatapultSubsystem} to use
     * @param powerLevel enum value for the piston power needed
     */
    public CatapultCommand(CatapultSubsystem catapult, CatapultPower powerLevel) {
        catapultSubsystem = catapult;
        power = powerLevel;

        addRequirements(catapultSubsystem);
    }

    //-------- METHODS --------\\

    @Override
    public void initialize() {
        if(power == CatapultPower.AllPistons){
            catapultSubsystem.extendAllPistons();
        }
        else if(power == CatapultPower.SmallPistons){
            catapultSubsystem.extendSmallPistons();
        }
        else if(power == CatapultPower.LargePistons){
            catapultSubsystem.extendLargePistons();
        }
        else if(power == CatapultPower.SetLongShot){
            catapultSubsystem.setLongShot();
        }
        else if(power == CatapultPower.SetShortShot){
            catapultSubsystem.setShortShot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        catapultSubsystem.retractAll();
    }

    // Enum for piston usage
    public static enum CatapultPower {
        SmallPistons, LargePistons, AllPistons, SetLongShot, SetShortShot;

        private CatapultPower() {
        }
    }
}
