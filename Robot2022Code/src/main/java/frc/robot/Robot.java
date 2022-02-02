// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** \mainpage FRC 930 Robot Code 2022
 * <h1>Robot Code 2022</h1>
 * <img src="logo.png" style="background-color:blue;padding:20px;width:300px" />
 * 
 * <p>This is the documentation automatically generated by doxygen for our 2022 robot code.</p>
 * <hr>
 * <p>Have a look at the <a href="annotated.html">classlist</a> for more detailed information about the project structure.</p>
 * <p>Our code is hosted at <a href="https://github.com/FRC930/Robot2022">Github</a>
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.ShuffleboardUtility;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  private CommandScheduler commandScheduler;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    commandScheduler = CommandScheduler.getInstance();
  }

  @Override
  public void robotPeriodic() {
    commandScheduler.run();

    ShuffleboardUtility.getInstance().update();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.beginAutoRunCommands();
    Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
    
  @Override
  public void teleopInit() {
    m_robotContainer.beginTeleopRunCommands();
  }

  @Override
  public void testInit() {

  }
  
  @Override
  public void testPeriodic() {

  }

}
