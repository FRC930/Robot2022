package frc.robot.utilities;

import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * ShooterUtility
 * Calculates how to get a shot in the hub
 */
public class ShooterUtility {

    // Maximum distance that you can shoot from in feet.
    private static double MAX_DISTANCE = 27;
    // Used Desmos to plot out best-fit lines
    // Link: https://www.desmos.com/calculator/mbuhfoziwi
    // Constants for speed best-fit lines
    // Y = MX + B
    private static double SPEED_M = 1.323076923;
    private static double TOP_SPEED_B = 65.48717949;
    private static double BOT_SPEED_B = 15.48717949;
    // Constants for hood best-fit line
    // Y = A(X^3) + B(X^2) + CX + D
    private static double HOOD_A = 0.13834842;
    private static double HOOD_B = -2.30271897;
    private static double HOOD_C = 15.14317395;
    private static double HOOD_D = -16.10579851;

    /**
     * <h3>setValuesToShuffleboard</h3>
     * Sends all three calculation results to the shuffleboard.
     * 
     * @param distance the distance from the hub in feet
     */
    public static void setValuesToShuffleboard(double distance) {
        // Applies each calculation to the shuffleboard
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.SHOOTER_BOTTOM_SPEED,
                new ShuffleBoardData<Double>(calculateBottomSpeed(distance)));
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.SHOOTER_TOP_SPEED,
                new ShuffleBoardData<Double>(calculateTopSpeed(distance)));
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.SHOOTER_HOOD_POSITION,
                new ShuffleBoardData<Double>(calculateHoodPos(distance)));
    }

    /**
     * <h3>calculateTopSpeed</h3>
     * Returns the new speed for the top roller to make the shot.
     * 
     * @param distance the distance from the hub in feet
     * @return the required speed in percent output
     */
    public static double calculateTopSpeed(double distance) {
        if (distance >= 0 && distance <= MAX_DISTANCE) {
            if (distance >= 8) {
                return (SPEED_M * distance + TOP_SPEED_B) / 100;
            } else if (distance >= 5) {
                return 0.76;
            } else {
                return 0.74;
            }
        } else {
            return -1;
        }
    }

    /**
     * <h3>calculateBottomSpeed</h3>
     * Returns the new speed for the bottom roller to make the shot.
     * 
     * @param distance the distance from the hub in feet
     * @return the required speed in percent output
     */
    public static double calculateBottomSpeed(double distance) {
        if (distance >= 0 && distance <= MAX_DISTANCE) {
            if (distance >= 8) {
                return (SPEED_M * distance + BOT_SPEED_B) / 100;
            } else if (distance >= 5) {
                return 0.26;
            } else {
                return 0.24;
            }
        } else {
            return -1;
        }
    }

    /**
     * <h3>calculateHoodPos</h3>
     * Returns the new hood positiion to make the shot.
     * 
     * @param distance the distance from the hub in feet
     * @return the required position in degrees
     */
    public static double calculateHoodPos(double distance) {
        if (distance >= 0 && distance <= MAX_DISTANCE) {
            if (distance >= 19) {
                return 30.0;
            } else if (distance >= 8) {
                return 28.44;
            } else {
                return HOOD_A * Math.pow(distance, 3) + HOOD_B * Math.pow(distance, 2) + HOOD_C * distance + HOOD_D;
            }
        } else {
            return -1;
        }
    }
}