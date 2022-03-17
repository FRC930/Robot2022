package frc.robot.utilities;

/**
 * ShooterUtility
 * Calculates how to get a shot in the hub
 */
public class ShooterUtility {

    // Flag for singleton
    private static ShooterUtility lastInstance = null;

    /**
     * <h3>getInstance</h3>
     * Methods to get singleton instance.
     * Creates one if never created.
     * 
     * @return the singleton object
     */
    public static ShooterUtility getInstance() {
        if (lastInstance == null) {
            lastInstance = new ShooterUtility();
        }
        return lastInstance;
    }

    // Used Desmos to plot out best-fit lines
    // Link: https://www.desmos.com/calculator/mbuhfoziwi
    // Constants for speed best-fit lines
    // Y = MX + B
    private double SPEED_M = 1.323076923;
    private double TOP_SPEED_B = 65.48717949;
    private double BOT_SPEED_B = 15.48717949;
    // Constants for hood best-fit line
    // Y = A(X^3) + B(X^2) + CX + D
    private double HOOD_A = 0.13834842;
    private double HOOD_B = -2.30271897;
    private double HOOD_C = 15.14317395;
    private double HOOD_D = -16.10579851;

    // Math variables
    private double launchAngle;
    private double launchSpeedTop;
    private double launchSpeedBottom;

    /**
     * Sets shooter math variables to -1
     */
    private ShooterUtility() {
        launchAngle = -1;
        launchSpeedTop = -1;
        launchSpeedBottom = -1;
    }

    /**
     * <h3>setShooterDistance</h3>
     * Sets the current shooter distance and calculates the shooter values.
     * 
     * @param distance to the hub in meters
     */
    public void setShooterDistance(double distance) {
        if (distance > 0 && metersToFeet(distance) <= 27) {
            launchAngle = calculateHoodPos(distance);
            launchSpeedTop = calculateTopSpeed(distance);
            launchSpeedBottom = calculateBottomSpeed(distance);
        } else {
            launchAngle = -1;
            launchSpeedTop = -1;
            launchSpeedBottom = -1;
        }
    }

    /**
     * <h3>calculateTopSpeed</h3>
     * Returns the new speed for the top roller to make the shot.
     * 
     * @param distance the distance from the hub
     * @return the required speed in percent output
     */
    private double calculateTopSpeed(double distance) {
        if (metersToFeet(distance) >= 8) {
            return (SPEED_M * distance + TOP_SPEED_B) / 100;
        } else if (metersToFeet(distance) >= 6) {
            return 0.76;
        } else if (metersToFeet(distance) >= 2) {
            return 0.74;
        } else {
            return 1.0;
        }
    }

    /**
     * <h3>calculateBottomSpeed</h3>
     * Returns the new speed for the bottom roller to make the shot.
     * 
     * @param distance the distance from the hub
     * @return the required speed in percent output
     */
    private double calculateBottomSpeed(double distance) {
        if (metersToFeet(distance) >= 8) {
            return (SPEED_M * distance + BOT_SPEED_B) / 100;
        } else if (metersToFeet(distance) >= 6) {
            return 0.26;
        } else if (metersToFeet(distance) >= 2) {
            return 0.24;
        } else {
            return 0.0;
        }
    }

    /**
     * <h3>calculateHoodPos</h3>
     * Returns the new hood positiion to make the shot.
     * 
     * @param distance the distance from the hub
     * @return the required position in degrees
     */
    private double calculateHoodPos(double distance) {
        if (metersToFeet(distance) >= 19) {
            return 30.0;
        } else if (metersToFeet(distance) >= 8) {
            return 28.44;
        } else {
            return HOOD_A * Math.pow(distance, 3) + HOOD_B * Math.pow(distance, 2) + HOOD_C * distance + HOOD_D;
        }
    }

    /**
     * <h3>getTopSpeed</h3>
     * Returns the speed for the top roller to make the shot.
     * 
     * @return the required speed in percent output
     */
    public double getTopSpeed() {
        return this.launchSpeedTop;
    }

    /**
     * <h3>getBottomSpeed</h3>
     * Returns the speed for the bottom roller to make the shot.
     * 
     * @return the required speed in percent output
     */
    public double getBottomSpeed() {
        return this.launchSpeedBottom;
    }

    /**
     * <h3>getHoodAngle</h3>
     * Returns the angle required to make the shot.
     * 
     * @return the required angle in degrees
     */
    public double getHoodAngle() {
        return this.launchAngle;
    }

    /**
     * <h3>metersToFeet</h3>
     * Converts distance meters to feet.
     * 
     * @param length in meters
     * @return the length in feet
     */
    public double metersToFeet(double length) {
        return 3.28084 * length;
    }
}