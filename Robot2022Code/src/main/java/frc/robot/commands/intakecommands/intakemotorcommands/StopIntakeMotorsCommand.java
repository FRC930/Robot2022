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

//-------- CLASS --------\\

/**
 * <h3>StopIntakeMotorsCommand</h3>
 * 
 * This class contols when the intake motors will stop
 */
public class StopIntakeMotorsCommand extends CommandBase {

  // -------- VARIABLES --------\\

  private final IntakeMotorSubsystem intakeMotorSubsystem;

  //-------- CONSTRUCTOR --------\\
   /**
   * <h3>CounterclockwiseIntakeMotorsCommand</h3>
   * 
   * This class contols whem the intake motors will stop
   * 
   * @param iMotors - Intake motors subsystem

   */
  public StopIntakeMotorsCommand(IntakeMotorSubsystem iMotors) {
    intakeMotorSubsystem = iMotors;
    addRequirements(iMotors); // Use addRequirements() here to declare subsystem dependencies.
  }

  //-------- METHODS --------\\
  
  /**
   * <h3>initialize</h3>
   * 
   * Called when the command is initially scheduled. 
   */
  @Override
  public void initialize() {
    intakeMotorSubsystem.setMotorSpeed(0.0);
  }
  
  /**
   * <h3>isFinished</h3>
   * 
   * Returns true when the command should end.
   * 
   * @return False
   */
  @Override
  public boolean isFinished() {
    return false;
  }

} // End of class StopIntakeMotorsCommand