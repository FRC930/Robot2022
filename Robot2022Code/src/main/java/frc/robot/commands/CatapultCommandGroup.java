
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CatapultCommand.CatapultPower;
import frc.robot.subsystems.CatapultSubsystem;

public class CatapultCommandGroup extends SequentialCommandGroup {

  /**
   * <h3>CatapultCommandGroup</h3>
   * 
   * Reusable sequential command group for managing the catapult fire delay,
   * launch, and retraction time.
   * 
   * @param catapultSubsystem - Catapult Subsystem
   * @param powerLevel        - Power level of the catapult launch
   * @param timeMS            - Time to retract.
   */
  public CatapultCommandGroup(CatapultSubsystem catapultSubsystem, CatapultPower powerLevel, double timeMS) {
    super(
        new WaitCommand(CatapultSubsystem.CATAPULT_FIRE_DELAY),
        new CatapultCommand(catapultSubsystem, CatapultPower.AllPistons)
            .withTimeout(timeMS));
  }
}
