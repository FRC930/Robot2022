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
 * <h3>RunIntakeMotorsCommand</h3>
 * 
 * This class contols the intake motors
 */

public class RunIntakeMotorsCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  private final double INTAKE_SPEED = 0.8;

  // -------- VARIABLES --------\\

  private final IntakeMotorSubsystem intakeMotorSubsystem;

  private boolean reversed = false;

  // -------- CONSTRUCTOR --------\\
  /**
   * <h3>RunIntakeMotorsCommand</h3>
   * 
   * This class contols the intake motors
   * 
   * @param iMotors    - Intake motors subsystem
   * @param isReversed
   */
  public RunIntakeMotorsCommand(IntakeMotorSubsystem iMotors, boolean isReversed) {
    intakeMotorSubsystem = iMotors;
    reversed = isReversed;

    addRequirements(iMotors); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- METHODS --------\\

  /**
   * <h3>initialize</h3>
   * 
   * Called when the command is initially scheduled
   * 
   * Sets the motor speed based on if its in reverse mode or not.
   */
  @Override
  public void initialize() {
    // Changes the direction of the motors based upon whether the driver is clicking
    // the reverse button or not
    if (!reversed) {
      intakeMotorSubsystem.setMotorSpeed(INTAKE_SPEED);
    } else {
      intakeMotorSubsystem.setMotorSpeed(-INTAKE_SPEED);
    }

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

  /**
   * <h3>end</h3>
   * 
   * Sets the motor speed for the intake to zero once the command ends.
   */
  @Override
  public void end(boolean interrupted) {
    intakeMotorSubsystem.setMotorSpeed(0.0);
  }

} // End of class RunIntakeMotorsCommand