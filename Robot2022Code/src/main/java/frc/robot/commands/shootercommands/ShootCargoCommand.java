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

    // -----VARIABLES----\\
    private final double bottomSpeed;
    private final double topSpeed;
    private final FlywheelSubsystem shooterSubsystem;
    private final IndexerMotorSubsystem indexerSubsystem;
    private int counter;

    /**
     * <h3>ShootCargoCommand</h3>
     * Uses shuffleboard speed if value is -1, otherwise use the values inputed.
     * 
     * @param shooter The ShooterSubsystem to use
     * @param speed   The speed(in PercentOutput) you want both wheels to be at. Set -1 to use shuffleboard values
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer, double speed) {
        this(shooter, indexer, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * Uses shuffleboard speed if value is -1, otherwise use the values inputed.
     * 
     * @param shooter     The ShooterSubsystem to use
     * @param topSpeed    The speed(in PercentOutput) you want the top wheel to be at. Set -1 to use shuffleboard
     * @param bottomSpeed The speed(in PercentOutput) you want the bottom wheel to be at. Set -1 to use shuffleboard
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer, double bottomSpeed,
            double topSpeed) {
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        this.bottomSpeed = bottomSpeed;
        this.topSpeed = topSpeed;
        counter = 0;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        if (bottomSpeed == -1) {
            // Sets speed from dashboard values
            shooterSubsystem.setBottomSpeed(SmartDashboard.getNumber("Shooter Bottom Speed", 0));
        } else if(bottomSpeed > 0) {
            // Sets using constructor speeds
            shooterSubsystem.setBottomSpeed(bottomSpeed);
        }
        else{
            // Stops motor
            shooterSubsystem.setTopSpeed(0);
        }
        if (topSpeed == -1) {
            // Sets speed from dashboard values
            shooterSubsystem.setTopSpeed(SmartDashboard.getNumber("Shooter Top Speed", 0));
        } else if(topSpeed > 0) {
            // Sets using constructor speeds
            shooterSubsystem.setTopSpeed(topSpeed);
        }
        else{
            // Stops motor
            shooterSubsystem.setTopSpeed(0);
        }
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
        if (counter == INDEXER_DELAY) {
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
