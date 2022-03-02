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
 * <h3> EndgameRotateHorizonalCommand </h3>
 * 
 * Rotates the endgame arm to be horizontal.
 */
public class EndgameRotateHorizonalCommand extends CommandBase {

  //-------- CONSTANTS --------\\

  private final double ARM_SPEED = 0.2;
  private final double HORIZONTAL_POSITION = 0;
  private final double DEADBAND = 0.1;

  //-------- VARIABLES --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;

  // -------- CONSTRUCTOR --------\\
  /**
   * <h3>EndgameRotateHorizontalCommand</h3>
   * 
   * Rotates the endgame arm to be horizontal.
   * 
   * @param motorSubsystem motor subsystem to control
   */
  public EndgameRotateHorizonalCommand(EndgameMotorSubsystem motorSubsystem) {
    m_MotorSubsystem = motorSubsystem;

    addRequirements(m_MotorSubsystem); // Use addRequirements() here to declare subsystem dependencies.
  }

  //-------- METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    if(m_MotorSubsystem.getArmRotation() < HORIZONTAL_POSITION){
      m_MotorSubsystem.setMotorSpeed(ARM_SPEED);
    }
    else {
      m_MotorSubsystem.setMotorSpeed(-ARM_SPEED);
    }

  }

  @Override
  public boolean isFinished(){ // when true, ends command
    return Math.abs(m_MotorSubsystem.getArmRotation() - HORIZONTAL_POSITION) < DEADBAND;
  }

  @Override
  public void end(boolean interrupted) {
    m_MotorSubsystem.setMotorSpeed(0.0);
  }

} // End of class EndgameRotateHorizontalCommandCommand