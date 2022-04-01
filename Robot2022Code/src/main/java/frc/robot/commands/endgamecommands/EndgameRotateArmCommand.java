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
 * <h3>EndgameRotateVerticalCommand</h3>
 * 
 * Rotates the endgame arm to preset positions.
 */
public class EndgameRotateArmCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  private final double APPROACH_POSITION = -0.25;
  private final double RESET_POSITION = 0;
  private final double DEADBAND = 0.025;

  // -------- VARIABLES --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;
  private final double target;

  // -------- CONSTRUCTOR --------\\
  /**
   * <h3>EndgameRotateVerticalCommand</h3>
   * 
   * Rotates the endgame arm to engage the Mid bar.
   * 
   * @param motorSubsystem motor subsystem to control
   */
  public EndgameRotateArmCommand(EndgameMotorSubsystem motorSubsystem, EndgamePosition position) {
    m_MotorSubsystem = motorSubsystem;
    if (position == EndgamePosition.ApproachPosition) {
      target = APPROACH_POSITION;
    } else if (position == EndgamePosition.ResetPosition) {
      target = RESET_POSITION;
    } else {
      target = 0;
    }
    addRequirements(m_MotorSubsystem); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    m_MotorSubsystem.setArmPosition(target);
  }

  @Override
  public boolean isFinished() { // when true, ends command
    return Math.abs(m_MotorSubsystem.getArmRotation() - target) < DEADBAND;
  }

  @Override
  public void end(boolean interrupted) {
    m_MotorSubsystem.setMotorSpeed(0.0);
  }

  // Enum for endgame positions
  public static enum EndgamePosition {
    ApproachPosition, ResetPosition;
  }

} // End of class EndgameRotateVerticalCommand