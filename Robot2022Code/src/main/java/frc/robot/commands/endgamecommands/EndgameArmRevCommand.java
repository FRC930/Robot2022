/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.EndgameMotorSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameArmRevCommand</h3>
 * 
 * Rotates the endgame arm backwards to reset the rotation
 */
public class EndgameArmRevCommand extends CommandBase {

  // -------- CONSTANTS --------\\
  private final double ARM_SPEED = -0.2;

  // -------- DECLARATIONS --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;

  // -------- CONSTRUCTOR --------\\
  /**
   * <h3>EndgameArmRevCommand</h3>
   * 
   * Rotates the endgame arm backwards to reset the rotation
   * 
   * @param motorSubsystem motor subsystem to control
   */
  public EndgameArmRevCommand(EndgameMotorSubsystem motorSubsystem) {
    m_MotorSubsystem = motorSubsystem;

    addRequirements(m_MotorSubsystem); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    m_MotorSubsystem.setMotorSpeed(ARM_SPEED);

  }

  @Override
  public boolean isFinished() { // when true, ends command
    return false;
  }

  @Override
  public void end(boolean interrupted) { // Interrupted when button is released
    m_MotorSubsystem.setMotorSpeed(0.0);
  }

} // End of class EndgameArmRevCommand