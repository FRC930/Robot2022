/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

// import java.util.logging.Logger;

import frc.robot.subsystems.EndgameMotorSubsystem;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameRotateVerticalCommand</h3>
 * 
 * Rotates the endgame arm to engage the Mid bar.
 */
public class EndgameRotateVerticalCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  // private static final Logger logger =
  // Logger.getLogger(EndgameRotateVertical.class.getName());
  // TODO: Establish speed for endgame arm
  private final double ARM_SPEED = 0.2;
  // TODO: WORK ON ENCODER VALUES
  private final double APPROACH_POSITION = 0;
  private final double DEADBAND = 100;

  // -------- DECLARATIONS --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;

  // -------- CONSTRUCTOR --------\\
  /**
   * Creates command
   * 
   * @param motorSubsystem motor subsystem to control
   */
  public EndgameRotateVerticalCommand(EndgameMotorSubsystem motorSubsystem) {
    m_MotorSubsystem = motorSubsystem;
    // logger.log(LOG_LEVEL_FINE, "Initializing the EndgameRotateVertical...");

    addRequirements(m_MotorSubsystem); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- COMMANDBASE METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    if (m_MotorSubsystem.getEncoderPosition() < APPROACH_POSITION) {
      m_MotorSubsystem.setMotorSpeed(ARM_SPEED);
    } else {
      m_MotorSubsystem.setMotorSpeed(-ARM_SPEED);
    }
    // logger.log(LOG_LEVEL_FINE, "Starting the arm motor (command)...");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_MotorSubsystem.getEncoderPosition() - APPROACH_POSITION) < DEADBAND;
  }

  @Override
  public void end(boolean interrupted) {
    m_MotorSubsystem.setMotorSpeed(0.0);
  }
} // End of class EndgameRotateVertical
