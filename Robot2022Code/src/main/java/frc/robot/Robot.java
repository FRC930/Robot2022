// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  private CommandScheduler commandScheduler;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    commandScheduler = CommandScheduler.getInstance();

    m_robotContainer.beginTeleopRunCommands();
  }

  @Override
  public void robotPeriodic() {
    commandScheduler.run();
  }
}
