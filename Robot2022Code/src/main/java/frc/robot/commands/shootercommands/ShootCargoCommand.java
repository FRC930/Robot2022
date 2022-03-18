/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.utilities.BallSensorUtility;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>ShootCargoCommand</h3>
 * 
 * Complete shooting command.
 */
public class ShootCargoCommand extends CommandBase {

    // -----CONSTANTS----\\
    // Number of cycles to wait before sending balls into shooter. (Cycles = time(in
    // seconds) / 0.02)
    private final int INDEXER_DELAY = 20;

    // -----VARIABLES----\\
    private final FlywheelSubsystem shooterSubsystem;
    private final IndexerMotorSubsystem indexerSubsystem;
    private boolean usingShuffleboard;
    private double bottomSpeed;
    private double topSpeed;
    private int counter;

    /**
     * <h3>ShootCargoCommand</h3>
     * Uses shuffleboard for speeds.
     * 
     * @param shooter The FlywheelSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer) {
        // ---CANNOT USE this() BECAUSE OF BOOLEAN FLAG---\\
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        usingShuffleboard = true;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The FlywheelSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     * @param speed   The speed(in PercentOutput) you want both wheels to spin at
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer, double speed) {
        // Applies speed to both motors
        this(shooter, indexer, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter     The FlywheelSubsystem to use
     * @param indexer     The IndexerMotorSubsystem to use
     * @param topSpeed    The speed(in PercentOutput) you want the top wheel to spin
     *                    at
     * @param bottomSpeed The speed(in PercentOutput) you want the bottom wheel to
     *                    spin at
     */
    public ShootCargoCommand(FlywheelSubsystem shooter, IndexerMotorSubsystem indexer, double bottomSpeed,
            double topSpeed) {
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        usingShuffleboard = false;
        this.bottomSpeed = bottomSpeed;
        this.topSpeed = topSpeed;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        // Gets values from shuffleboard driver tab
        if (usingShuffleboard) {
            this.bottomSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_BOTTOM_SPEED).getData();
            this.topSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_TOP_SPEED).getData();
        }

        shooterSubsystem.setBottomSpeed(bottomSpeed);
        shooterSubsystem.setTopSpeed(topSpeed);
        counter = 0;
    }

    @Override
    public void execute() {
        counter++;
        // Waits for delay before activating indexer system
        if (counter == INDEXER_DELAY) {
            indexerSubsystem.setIntakeMotorSpeed(1.0);
            indexerSubsystem.setLoadedMotorSpeed(1.0);
        }
    }

    @Override
    public boolean isFinished() {
        // Finishes if no balls are left
        return !BallSensorUtility.getInstance().intakeIsTripped() &&
                !BallSensorUtility.getInstance().loadedIsTripped();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setBottomSpeed(0);
        shooterSubsystem.setTopSpeed(0);
        indexerSubsystem.setIntakeMotorSpeed(0);
        indexerSubsystem.setLoadedMotorSpeed(0);
    }
} // End of CLass
