/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

// import java.util.logging.Logger;

import frc.robot.subsystems.EndgameMotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

//-------- COMMAND CLASS --------\\

public class RotateArmCommand extends CommandBase {

  //-------- CONSTANTS --------\\

  // private static final Logger logger = Logger.getLogger(RotateArmCommand.class.getName());
  private static double ARM_SPEED;

  //-------- DECLARATIONS --------\\

  private final EndgameMotorSubsystem m_MotorSubsystem;
  

  //-------- CONSTRUCTOR --------\\

  public RotateArmCommand(EndgameMotorSubsystem motorSubsystem) {
    m_MotorSubsystem = motorSubsystem;
    // logger.log(LOG_LEVEL_FINE, "Initializing the RotateArmCommand...");

    addRequirements(motorSubsystem);  // Use addRequirements() here to declare subsystem dependencies.
  }

  //-------- COMMANDBASE METHODS --------\\

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
     m_MotorSubsystem.setMotorSpeed(ARM_SPEED);
     
    // logger.log(LOG_LEVEL_FINE, "Starting the arm motor (command)..."); 
  }
  
  @Override   // Returns true when the command should end.
  public boolean isFinished() {
    return true;
  }

} // End of class RotateArmCommand
