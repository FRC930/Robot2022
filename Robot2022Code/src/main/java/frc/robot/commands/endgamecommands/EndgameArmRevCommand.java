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
 * <h3> EndgameArmRevCommand </h3>
 * 
 * Rotates the endgame arm backwards to reset the rotation.
 */
public class EndgameArmRevCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  // private static final Logger logger =
  // Logger.getLogger(EndgameArmRevCommand.class.getName());
  // TODO: Establish speed for endgame arm
  private final double ARM_SPEED = -0.2;

  // -------- DECLARATIONS --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;

  // -------- CONSTRUCTOR --------\\
  /**
   * Creates command
   * @param motorSubsystem motor subsystem to control
   */
  public EndgameArmRevCommand(EndgameMotorSubsystem motorSubsystem) {
    m_MotorSubsystem = motorSubsystem;
    // logger.log(LOG_LEVEL_FINE, "Initializing the EndgameArmRevCommand...");

    addRequirements(m_MotorSubsystem); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- COMMANDBASE METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    m_MotorSubsystem.setMotorSpeed(ARM_SPEED);

    // logger.log(LOG_LEVEL_FINE, "Starting the arm motor (command)...");
  }

  @Override
  public void execute(){
    m_MotorSubsystem.getArmRotation();
  }
  

  @Override
  public boolean isFinished() { // when true, ends command
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_MotorSubsystem.setMotorSpeed(0.0);
  }
} // End of class EndgameArmRevCommand
