/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * <h3>ShootCargoCommand</h3>
 * 
 * Shoots ball with shooter.
 */
public class ShootCargoCommand extends CommandBase {

    private final double bottomSpeed;
    private final double topSpeed;
    private final FlywheelSubsystem shooterSubsystem;

        /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The ShooterSubsystem to use
     * @param speed The percent output you want both wheels to be at
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, double speed)
    {
        this(shooter, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The ShooterSubsystem to use
     * @param topSpeed The percent output you want the top wheel to be at
     * @param bottomSpeed The percent output you want the bottom wheel to be at
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, double bottomSpeed, double topSpeed)
    {
        shooterSubsystem = shooter;
        this.bottomSpeed = bottomSpeed;
        this.topSpeed = topSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setTopSpeed(bottomSpeed);
        shooterSubsystem.setTopSpeed(topSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

} // End of CLass
