/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.intakecommands.intakemotorcommands;
import frc.robot.subsystems.BallSensorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

//-------- COMMAND CLASS --------\\
/**
 * <h3>CounterclockwiseIntakeMotorsCommand</h3>
 * 
 * This class contols the counter-clockwise intake motors
 */
public class RunIntakeMotorsCommand extends CommandBase {

  // -------- CONSTANTS --------\\

  private final double INTAKE_SPEED = 0.8;

  // -------- VARIABLES --------\\

  private final IntakeMotorSubsystem intakeMotors;
  private final BallSensorSubsystem ballSensor;
  private boolean reversed = false;

  //-------- CONSTRUCTOR --------\\
   /**
   * <h3>CounterclockwiseIntakeMotorsCommand</h3>
   * 
   * This class contols the counter-clockwise intake motors
   * 
   * @param iMotors - Intake motors subsystem
   */
  public RunIntakeMotorsCommand(IntakeMotorSubsystem iMotors, BallSensorSubsystem bSensor, boolean isReversed) {
    intakeMotors = iMotors;
    ballSensor = bSensor;
    reversed = isReversed;

    addRequirements(iMotors); // Use addRequirements() here to declare subsystem dependencies.
  }

  // -------- COMMANDBASE METHODS --------\\

  /**
  * Called when the command is initially scheduled. 
  */
  @Override  
  public void initialize() {
    if(!ballSensor.isTripped()){
      if(!reversed) {
        intakeMotors.setMotorSpeed(INTAKE_SPEED);
      } else {
        intakeMotors.setMotorSpeed(-INTAKE_SPEED);
      }
    }
  }
  
  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return ballSensor.isTripped();
  }

  @Override
  public void end(boolean interrupted){
    intakeMotors.setMotorSpeed(0.0);
  }

} // End of class RunIntakeMotorsCommand