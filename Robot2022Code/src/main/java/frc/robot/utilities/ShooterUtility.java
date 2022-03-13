package frc.robot.utilities;

/**
 * ShooterSubsystem
 * Calculates how to get a shot in the hub
 */
public class ShooterUtility {

    // All measurements are in meters
    private final double BALL_RADIUS = 0.12065;
    private final double HUB_DIAMETER = 1.2192;
    private final double HUB_HEIGHT = 2.6416;
    private final double START_HEIGHT = 0.7366;
    private final double HEIGHT_DIFFERENCE = HUB_HEIGHT - START_HEIGHT;
    private static final double TOP_WHEEL_RADIUS = 0.0254;
    private static final double BOTTOM_WHEEL_RADIUS = 0.0508;
    private final double GRAVITY = 9.8;

    // Flag for singleton
    private static ShooterUtility lastInstance = null;

    /**
     * Methods to get instance from singleton
     * this one uses the default constructed object, this is the one to use when getting single instance
     * If lastInstance doesnt exist create it
     * @return
     */
    public static ShooterUtility getInstance() {
        if (lastInstance == null) {
            lastInstance = new ShooterUtility();
        }
        return lastInstance;
    }

    // Math variables
    private double launchAngle;
    private double launchVelocity;
    private double distanceToHub;

    /**
     * Sets shooter math variables to -1
     */
    private ShooterUtility() {
        launchAngle = -1;
        launchVelocity = -1;
        distanceToHub = -1;
    }

    /**
     * Returns the velocity required to make the shot. Will return -1 if error
     * occurs.
     * 
     * @return {@link #velocity} set in {@link #calculateVelocity}
     */
    public double getVelocity() {
        return this.launchVelocity;
    }

    /**
     * Returns whether the inner shot is possible
     * 
     * @return {@link #shotOutcome} set in {@link #calculateTrajectory}
     */
    public boolean getPossibleShot() {
        return true;
    }
}