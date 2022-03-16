/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;

/**
 * <h3>ShootCargoCommand</h3>
 * 
 * Shoots ball with shooter.
 */
public class ShootCargoCommand extends CommandBase {

    // -----CONSTANTS----\\
    // Number of cycles to wait before sending balls into shooter. (Cycles = time(in
    // seconds) / 0.02)
    private final int INDEXER_DELAY = 20;
    private final double bottomSpeed;
    private final double topSpeed;
    private final FlywheelSubsystem shooterSubsystem;
    private final IndexerMotorSubsystem indexerSubsystem;

    private int counter;

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The ShooterSubsystem to use
     * @param speed   The velocity(in RPMs) you want both wheels to be at
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer, double speed) {
        this(shooter, indexer, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter     The ShooterSubsystem to use
     * @param topSpeed    The velocity(in RPMs) you want the top wheel to be at
     * @param bottomSpeed The velocity(in RPMs) you want the bottom wheel to be at
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer, double bottomSpeed,
            double topSpeed) {
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        this.bottomSpeed = bottomSpeed;
        this.topSpeed = topSpeed;
        counter = 0;
        SmartDashboard.putNumber("Shoot Delay", INDEXER_DELAY);
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        // shooterSubsystem.setBottomSpeed(bottomSpeed);
        // shooterSubsystem.setTopSpeed(topSpeed);
        // Sets speed from dashboard values
        shooterSubsystem.setBottomSpeed(SmartDashboard.getNumber("Shooter Bottom Speed", 0));
        shooterSubsystem.setTopSpeed(SmartDashboard.getNumber("Shooter Top Speed", 0));
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
        if (counter == SmartDashboard.getNumber("Shoot Delay", 0.0)) {
            indexerSubsystem.setIntakeMotorSpeed(1.0);
            indexerSubsystem.setLoadedMotorSpeed(1.0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setBottomSpeed(0);
        shooterSubsystem.setTopSpeed(0);
        indexerSubsystem.setIntakeMotorSpeed(0);
        indexerSubsystem.setLoadedMotorSpeed(0);
    }
} // End of CLass
