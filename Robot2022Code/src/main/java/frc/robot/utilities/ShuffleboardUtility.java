//----- IMPORTS -----\\

package frc.robot.utilities;


import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

//----- CLASS -----\\
/**
 * <h3>ShuffleboardUtility</h3>
 * 
 * ShuffleboardUtility represents the shuffleboard to our code
 * 
 * @author Alexander Taylor
 * @since 26 January 2022
 * @version 1.0
 */
public class ShuffleboardUtility {

    // ----- VARIABLES -----\\

    private static ShuffleboardUtility instance;

    private Map<ShuffleboardKeys, MapData> shuffleboardMap;

    private SendableChooser<Command> autonChooser;

    private final boolean IS_DEBUGGING = false;

    // ----- TABS -----\\

    public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Tab");
    public static final ShuffleboardTab testingTab = Shuffleboard.getTab("Testing Tab");

    // ----- CONSTRUCTOR -----\\

    private ShuffleboardUtility() {
        shuffleboardMap = new HashMap<>();

        autonChooser = new SendableChooser<>();

        driverTab.add("Auton Path Selector", autonChooser);
    }

    // ----- METHOD(S) -----\\

    /**
     * <h3>getInstance</h3>
     * 
     * ShuffleboardUtility is a singleton, so getInstance returns the instance of
     * the class that the program will use
     * 
     * @return the instance
     */
    public static ShuffleboardUtility getInstance() {
        if (instance == null) {
            instance = new ShuffleboardUtility();
        }
        return instance;
    }

    /**
     * <h3>putToDriverTab</h3>
     * 
     * Receives data then inserts it into storage and the shuffleboard widgets on
     * driver tab.
     * 
     * @param key
     * @param data
     */
    public void putToShuffleboard(ShuffleboardTab tab, ShuffleboardKeys key, ShuffleBoardData<?> data) {
        if (IS_DEBUGGING || !tab.equals(testingTab)) {
            // Check to see if we have to add a new widget to the Shuffleboard tab
            if (shuffleboardMap.containsKey(key)) {
                // If the widget exists, simply update it to the new value
                shuffleboardMap.put(key, new MapData(data, shuffleboardMap.get(key).m_entry));
            } else {
                // Since the widget doesn't exist, we need to create a new entry for it
                shuffleboardMap.put(key, new MapData(data, tab.add(key.m_name, data.m_data).getEntry()));
            }
        }
    }

    /**
     * <h3>update</h3>
     * 
     * Updates all of the entries on shuffleboard
     * 
     * Gets called in robotPeriodic
     */
    public void update() {
        MapData data;
        for (ShuffleboardKeys currentKey : shuffleboardMap.keySet()) {
            data = shuffleboardMap.get(currentKey);
            // Sets the network table entry
            data.m_entry.setValue(data.m_dataContainer.m_data);
        }
    }

    /**
     * <h3>addAutonOptions</h3>
     * 
     * Adds an option for auton selection
     * 
     * @param pathName    name of the path added
     * @param autoCommand instance of the command being added
     */
    public void addAutonOptions(String pathName, CommandBase autoCommand) {
        autonChooser.addOption(pathName, autoCommand);
    }

    /**
     * <h3>getSelectedAutonPath</h3>
     * 
     * Get the option selected on the Shuffleboard
     * 
     * @return the selected auton path
     */
    public Command getSelectedAutonPath() {
        return autonChooser.getSelected();
    }

    /**
     * <h3>setDefaultAutonOptions</h3>
     * 
     * Sets the default option for auton selection
     * 
     * @param pathName    name of the path added
     * @param autoCommand instance of the command being added
     */
    public void setDefaultAutonOptions(String pathName, CommandBase autoCommand) {
        autonChooser.setDefaultOption(pathName, autoCommand);
    }

    // ----- ENUM KEYS -----\\
    /**
     * <h3>ShuffleboardKeys</h3>
     * 
     * Represents the keys for putting data to shuffleboard
     */
    public static enum ShuffleboardKeys {

        // BALL MANAGEMENT
        CATAPULT_SENSOR("Catapult Sensor"),
        ALLIANCE_COLOR("Alliance Color"),
        LED_PATTERNS("LED Patterns"),
        AUTONOMOUS_PATH("Autonomous Paths"),
        INTAKE_SENSOR("Intake Sensor"),
        INTAKE_POSITIONING("Intake Positioning"),
        INTAKE_DIRECTION("Intake Direction"),
        // CATAPULT_LOADED("Catapult loaded"),
        INTAKE_DOWN("Intake down"),

        // ENDGAME

        ENDGAME_SENSOR1("Endgame Sensor 1"),
        ENDGAME_SENSOR2("Endgame Sensor 2"),
        ENDGAME_SENSOR3("Endgame Sensor 3"),
        ENDGAME_SENSOR4("Endgame Sensor 4"),
        ENDGAME_ENCODER("Endgame Encoder"),
        PISTON_SENSOR("Piston Sensor"),

        // DRIVE TRAIN
        LEFT_SPEED("Speed of left drivetrain"),
        RIGHT_SPEED("Speed of right drivetrain"),
        DRIVETRAIN_SHIFTED("Drivetrain shifted"),

        // MISCELLANEOUS
        CAMERA_STREAM("Camera stream"),
        DISTANCE_FROM_GOAL("Distance from goal");

        final String m_name;

        ShuffleboardKeys(String name) {
            m_name = name;
        }
    }

    // ----- STRUCT(S) -----\\
    /**
     * <h3>ShuffleboardData</h3>
     * 
     * A template class for storing the data that we will put to shuffleboard
     * 
     * T is a generic type that could be anything
     */
    public static class ShuffleBoardData<T> {
        private final T m_data;

        public ShuffleBoardData(T data) {
            m_data = data;
        }
    }

    /**
     * <h3>MapData</h3>
     * 
     * Stores a pair of a data and a network table entry
     */
    private static class MapData {
        // A generic data entry
        public final ShuffleBoardData<?> m_dataContainer;
        // The network table entry that we will use to
        public final NetworkTableEntry m_entry;

        public MapData(ShuffleBoardData<?> data, NetworkTableEntry entry) {
            m_dataContainer = data;
            m_entry = entry;
        }
    }
}