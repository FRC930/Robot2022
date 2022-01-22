/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.intakecommands.intakemotorcommands;
import frc.robot.subsystems.IntakeMotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

//-------- COMMAND CLASS --------\\
/**
 * <h3>CounterclockwiseIntakeMotorsCommand</h3>
 * 
 * This class contols the counter-clockwise intake motors
 */
public class CounterclockwiseIntakeMotorsCommand extends CommandBase {

  //-------- CONSTANTS --------\\

  private static final double INTAKE_SPEED = -0.6;  

  //-------- DECLARATIONS --------\\

  private final IntakeMotorSubsystem intakeMotors;

  //-------- CONSTRUCTOR --------\\
   /**
   * <h3>CounterclockwiseIntakeMotorsCommand</h3>
   * 
   * This class contols the counter-clockwise intake motors
   * 
   * @param iMotors - Intake motors subsystem
   */
  public CounterclockwiseIntakeMotorsCommand(IntakeMotorSubsystem iMotors) {
    intakeMotors = iMotors;

    addRequirements(iMotors);  // Use addRequirements() here to declare subsystem dependencies.
  }

  //-------- COMMANDBASE METHODS --------\\

  /**
  * Called when the command is initially scheduled. 
  */
  @Override  
  public void initialize() {
     intakeMotors.setMotorSpeed(INTAKE_SPEED);
     
  }
  
  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return true;
  }

} //End of class RunIntakeMotorsCommand