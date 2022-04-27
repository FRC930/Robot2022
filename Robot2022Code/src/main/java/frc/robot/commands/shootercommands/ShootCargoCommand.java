/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//----- IMPORTS -----\\

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.ShuffleboardUtility;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

//----- CLASS -----\\
/**
 * <h3>ShootCargoCommand</h3>
 * 
 * Complete shooting command.
 */

 // Overloaded constructors
public class ShootCargoCommand extends CommandBase {

    // -----CONSTANTS----\\

    public static final double TELEOP_SHOOT_TIME = 5.0;
    // Number of cycles to wait before sending balls into shooter. (Cycles = time(in seconds) / 0.02)
    private final int INDEXER_DELAY = 20;

    // -----VARIABLES----\\

    private final ShooterSubsystem shooterSubsystem;
    private final IndexerMotorSubsystem indexerSubsystem;
    private boolean usingShuffleboard;
    private double bottomSpeed;
    private double topSpeed;
    private int counter;

    //----- CONSTRUCTOR(S) -----\\

    /**
     * <h3>ShootCargoCommand</h3>
     * Uses shuffleboard for speeds.
     * 
     * @param shooter The ShooterSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     */
    public ShootCargoCommand(ShooterSubsystem shooter, IndexerMotorSubsystem indexer) {
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        usingShuffleboard = true;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter The ShooterSubsystem to use
     * @param indexer The IndexerMotorSubsystem to use
     * @param speed   The speed(in PercentOutput) you want both wheels to spin at
     */
    public ShootCargoCommand(ShooterSubsystem shooter, IndexerMotorSubsystem indexer, double speed) {
        // Applies speed to both motors
        this(shooter, indexer, speed, speed);
    }

    /**
     * <h3>ShootCargoCommand</h3>
     * 
     * @param shooter     The ShooterSubsystem to use
     * @param indexer     The IndexerMotorSubsystem to use
     * @param topSpeed    The speed(in PercentOutput) you want the top wheel to spin
     *                    at
     * @param bottomSpeed The speed(in PercentOutput) you want the bottom wheel to
     *                    spin at
     */
    public ShootCargoCommand(ShooterSubsystem shooter, IndexerMotorSubsystem indexer, double topSpeed,
            double bottomSpeed) {
        shooterSubsystem = shooter;
        indexerSubsystem = indexer;
        usingShuffleboard = false;
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        addRequirements(shooterSubsystem, indexerSubsystem);
    }

    //----- METHODS -----\\

    /**
     * <h3>initialize</h3>
     */
    @Override
    public void initialize() {
        // Gets values from shuffleboard driver tab
        if (usingShuffleboard) {
            this.bottomSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_BOTTOM_SPEED).getData();
            this.topSpeed = (double) ShuffleboardUtility.getInstance().getFromShuffleboard(
                    ShuffleboardKeys.SHOOTER_TOP_SPEED).getData();
        } else {
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_BOTTOM_SPEED, new ShuffleBoardData<Double>(bottomSpeed));
            ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                    ShuffleboardKeys.SHOOTER_TOP_SPEED, new ShuffleBoardData<Double>(topSpeed));
        }

        shooterSubsystem.setBottomSpeed(bottomSpeed);
        shooterSubsystem.setTopSpeed(topSpeed);
        counter = 0;
    }

    /**
     * <h3>execute</h3>
     */
    @Override
    public void execute() {
        // Increments the counter every 20 ms
        counter++;
        // Waits for delay before activating indexer system
        if (counter == INDEXER_DELAY) {
            indexerSubsystem.setStagedMotorSpeed(1.0);
            indexerSubsystem.setLoadedMotorSpeed(1.0);
        }
    }

    /**
     * <h3>isFinished</h3>
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * <h3>end</h3>
     */
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopMotors();
        indexerSubsystem.stopMotors();
    }
} // End of CLass
