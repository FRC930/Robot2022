/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.intakecommands.intakemotorcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeMotorSubsystem;

//-------- COMMAND CLASS --------\\

public class CounterclockwiseIntakeMotorsCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  private static final double INTAKE_SPEED = -0.6;

  // -------- DECLARATIONS --------\\

  private final IntakeMotorSubsystem intakeMotors;

  // -------- CONSTRUCTOR --------\\

  public CounterclockwiseIntakeMotorsCommand(IntakeMotorSubsystem iMotors) {
    intakeMotors = iMotors;

    addRequirements(iMotors); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- COMMANDBASE METHODS --------\\

  @Override // Called when the command is initially scheduled.
  public void initialize() {
    intakeMotors.setMotorSpeed(INTAKE_SPEED);

  }

  @Override // Returns true when the command should end.
  public boolean isFinished() {
    return true;
  }

} // End of class RunIntakeMotorsCommand