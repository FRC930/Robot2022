/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.triggers;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

//-------- TRIGGER CLASS --------\\

/**
 * <h3>POVTrigger</h3>
 * 
 * POVTrigger is used to create a trigger for events dealing with the d-pad
 */
public class POVTrigger extends Trigger {

  // -------- DECLARATIONS --------\\

  private XboxController controller;
  private int povAxis;
  private int povValue;

  //-------- CONSTRUCTOR --------\\

  /**
   * Creates a joystick button for triggering commands.
   *
   * @param controller     The controller object that has an axis
   * 
   * @param povValue The direction number (see {@link edu.wpi.first.wpilibj.Joystick#getRawAxis(int) getRawAxis})
   */
  public POVTrigger(XboxController controller, int povAxis, int povValue) {
    requireNonNullParam(controller, "joystick", "POVTrigger");

    this.controller = controller;
    this.povAxis = povAxis;
    this.povValue = povValue;
  }

  //-------- METHODS --------\\

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return (this.controller.getPOV(this.povAxis) == povValue);
  }

} // end of class AxisTrigger