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
 * <h3>EndgameArmCommand</h3>
 * 
 * Rotates the endgame arm forward to climb the hangar
 */
public class EndgameArmCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  private double armSpeed;

  // -------- VARIABLES --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;

  // -------- CONSTRUCTOR --------\\

  /**
   * <h3>EndgameArmCommand</h3>
   * 
   * Rotates the endgame arm forward to climb the hangar
   * 
   * @param motorSubsystem  - Arm motors
   * @param speed           - Speed of arm rotation
   */
  public EndgameArmCommand(EndgameMotorSubsystem motorSubsystem, double speed) {
    m_MotorSubsystem = motorSubsystem;
    armSpeed = speed;

    addRequirements(m_MotorSubsystem); // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * <h3>EndgameArmCommand</h3>
   * 
   * Rotates the endgame arm forward to climb the hangar
   * 
   * @param motorSubsystem  - Arm motors
   * @param speed           - Speed of arm rotation, currently set to 0.5 by default (use overload to specify speed)
   */
  public EndgameArmCommand(EndgameMotorSubsystem motorSubsystem) {
    this(motorSubsystem, 0.8);
  }

  // -------- METHODS --------\\
  /**
   * Runs when the command is started
   */
  @Override // Called when the command is initially scheduled.
  public void initialize() {
    // Sets speed of motor to the constant
    m_MotorSubsystem.setMotorSpeed(armSpeed);

  }

  // isFinished() is left false on purpose. Needs to be interrupted.
  @Override
  public boolean isFinished() { // when true, ends command
    return false;
  }

  @Override
  public void end(boolean interrupted) { // Interrupted when button is released
    // Sets the motor speed to 0 to stop the motor because command is done
    m_MotorSubsystem.setMotorSpeed(0.0);
  }

} // End of class EndgameArmCommand