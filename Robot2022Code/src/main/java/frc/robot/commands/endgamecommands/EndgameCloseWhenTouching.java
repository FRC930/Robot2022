/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//-------- IMPORTS --------\\

package frc.robot.commands.endgamecommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndgamePistonSubsystem;
import frc.robot.utilities.EndgameSensorUtility;

//-------- COMMAND CLASS --------\\
/**
 * <h3>EndgameCloseWhenTouching</h3>
 * 
 * Closes a claw when the sensor is active
 */
public class EndgameCloseWhenTouching extends CommandBase {

    // -------- VARIABLES --------\\
    private final EndgamePistonSubsystem endgamePiston;
    private final EndgameSensorPairs sensor;
    private Debouncer debouncer = null;
    private final EndgameSensorUtility sensorUtility = EndgameSensorUtility.getInstance();

    // -------- CONSTRUCTOR --------\\

    /**
     * <h3>EndgameCloseWhenTouching</h3>
     * 
     * Closes a claw when the sensor is active. Will not use a debouncer
     * 
     * @param _endgamePiston piston of claw to be closed
     * @param sensorPair     sensor pair used to detect
     */
    public EndgameCloseWhenTouching(EndgamePistonSubsystem _endgamePiston, EndgameSensorPairs sensorPair) {
        this(_endgamePiston, sensorPair, 0.0);
    }

    /**
     * <h3>EndgameCloseWhenTouching</h3>
     * 
     * Closes a claw when the sensor is active. Can set a debouncer for rising.
     * 
     * @param _endgamePiston piston of claw to be closed
     * @param sensorPair     sensor pair used to detect
     * @param debounceTime   time to apply to debouncer. Use 0 to not use the
     *                       debouncer.
     */
    public EndgameCloseWhenTouching(EndgamePistonSubsystem _endgamePiston, EndgameSensorPairs sensorPair,
            double debounceTime) {
        sensor = sensorPair;
        endgamePiston = _endgamePiston;
        if (debounceTime > 0) {
            debouncer = new Debouncer(debounceTime, DebounceType.kRising);
        }
        addRequirements(endgamePiston);
    }

    // -------- METHODS --------\\

    @Override
    public boolean isFinished() { // returns true when the sensor is active
        if (sensor == EndgameSensorPairs.SensorPair2) {
            return getSensorPair2Touching();
        } else if (sensor == EndgameSensorPairs.SensorPair4) {
            return getSensorPair4Touching();
        } else {
            return true;
        }
    }

    private boolean getSensorPair2Touching() {
        boolean state = sensorUtility.left2IsTouching() &&
                sensorUtility.right2IsTouching();
        return (debouncer != null) ? debouncer.calculate(state) : state;
    }

    private boolean getSensorPair4Touching() {
        boolean state = sensorUtility.left4IsTouching() &&
                sensorUtility.right4IsTouching();
        return (debouncer != null) ? debouncer.calculate(state) : state;
    }

    @Override // Interrupted when button is released
    public void end(boolean interuppted) { // closes the claw
        endgamePiston.closed();
    }

    // Enum for endgame positions, sensor pairs are the claws next to eachother
    public static enum EndgameSensorPairs {
        SensorPair2, SensorPair4;
    }

} // End of class EndgameCloseWhenTouching