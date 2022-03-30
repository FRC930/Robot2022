//----- IMPORTS -----\\

package frc.robot.utilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
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
    private Map<ShuffleboardKeys, MapData> pastDataMap;

    private SendableChooser<Command> autonChooser;
    private SendableChooser<Integer> pipelineChooser;

    private final boolean IS_DEBUGGING = false;

    // ----- TABS -----\\

    public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Tab");
    public static final ShuffleboardTab testingTab = Shuffleboard.getTab("Testing Tab");

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable driverTable = inst.getTable("Shuffleboard");
    private NetworkTableEntry currentChooserValue = driverTable.getEntry("Driver Tab/Pipeline Selector/selected");
    private HashMap<String, Integer> pipelineMap = new HashMap<>();

    // ----- CONSTRUCTOR -----\\

    private ShuffleboardUtility() {
        shuffleboardMap = new HashMap<>();
        pastDataMap = new HashMap<>();

        autonChooser = new SendableChooser<>();

        pipelineChooser = new SendableChooser<>();

        {
            // Get the current pipeline file so we can reset it from what it was last time
            // we ran
            File currentPipeline = new File(
                    Filesystem.getOperatingDirectory().getAbsolutePath() + "/currentPipeline.txt");

            // Check to make sure that the file exists
            if (currentPipeline.exists()) {
                // Start reading the file
                try (Scanner reader = new Scanner(currentPipeline)) {
                    String pipelineName = reader.nextLine();
                    int pipelineIndex = Integer.parseInt(reader.nextLine());
                    // Set the default options for the pipeline chooser
                    pipelineChooser.setDefaultOption(pipelineName, pipelineIndex);
                } catch (IOException e) {
                    System.out.println("****** COULDN\'T FIND CURRENT PIPELINE FILE ******");
                }
            } else {
                try {
                    // Create a new file to write data
                    currentPipeline.createNewFile();

                    // Write the default settings
                    FileWriter writer = new FileWriter(currentPipeline);
                    writer.write("(Default)\n");
                    writer.write("0");
                    writer.close();
                } catch (IOException e) {
                }

                pipelineChooser.setDefaultOption("(Default)", 0);
            }
        }

        // Set up the listener for the network table entry
        driverTable.getEntry("Driver Tab/Pipeline Selector/selected").addListener((EntryNotification notif) -> {
            // Get the file in which we stored the current pipeline
            File currentPipeline = new File(
                    Filesystem.getOperatingDirectory().getAbsolutePath() + "/currentPipeline.txt");

            // Make sure the file is empty
            if (currentPipeline.exists()) {
                currentPipeline.delete();
                try {
                    currentPipeline.createNewFile();
                } catch (IOException e) {
                }
            }

            // Set up the file writer
            FileWriter writer;
            try {
                // Write the pipeline that on the Shuffleboard to the file
                writer = new FileWriter(currentPipeline);
                String pipelineName = currentChooserValue.getString("(Default)");
                writer.write(pipelineName + "\n");
                writer.write(Integer.toString(pipelineMap.get(pipelineName)));
                writer.close();

                PhotonVisionUtility.getInstance().setPiCameraPipeline(pipelineMap.get(pipelineName));
            } catch (IOException e) {
            }
        }, EntryListenerFlags.kUpdate);

        driverTab.add("Auton Path Selector", autonChooser);

        driverTab.add("Pipeline Selector", pipelineChooser);
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
     * <h3>getFromShuffleboard</h3>
     * 
     * This method gets a value from the shuffleboard
     * 
     * @param key the value to get from the shuffleboard
     * @return the value that the shuffleboard has for the key
     */
    public ShuffleBoardData<?> getFromShuffleboard(ShuffleboardKeys key) {
        if (shuffleboardMap.containsKey(key)) {
            return shuffleboardMap.get(key).m_dataContainer;
        } else {
            return null;
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
            if (pastDataMap.containsKey(currentKey)) {
                if (!pastDataMap.get(currentKey).equals(shuffleboardMap.get(currentKey))) {
                    data = shuffleboardMap.get(currentKey);
                    // Sets the network table entry
                    data.m_entry.setValue(data.m_dataContainer.m_data);

                    pastDataMap.put(currentKey, data);
                }
            } else {
                pastDataMap.put(currentKey, shuffleboardMap.get(currentKey));
            }
        }

    }

    /**
     * <h3>addPipelineChooser</h3>
     * 
     * Add a value to the pipeline chooser
     * 
     * @param displayName the name to display
     * @param value       the index of the pipeline
     */
    public void addPipelineChooser(String displayName, int value) {
        pipelineChooser.addOption(displayName, value);
        pipelineMap.put(displayName, value);
    }

    /**
     * <h3>getSelectedPipelineChooser</h3>
     * 
     * Gets the pipeline that we have selected in the shuffleboard
     * 
     * @return the index of the wanted pipeline
     */
    public int getSelectedPipelineChooser() {
        return pipelineChooser.getSelected();
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

        // SHOOTER MANAGEMENT
        SHOOTER_TOP_SPEED("Shooter Top Speed"),
        SHOOTER_BOTTOM_SPEED("Shooter Bottom Speed"),
        SHOOTER_HOOD_POSITION("Hood Position"),

        // BALL MANAGEMENT
        LOADED_SENSOR("Loaded Sensor"),
        ALLIANCE_COLOR("Alliance Color"),
        LED_PATTERNS("LED Patterns"),
        AUTONOMOUS_PATH("Autonomous Paths"),
        INTAKE_SENSOR("Intake Sensor"),
        INTAKE_POSITIONING("Intake Positioning"),
        INTAKE_DIRECTION("Intake Direction"),
        INTAKE_DOWN("Intake down"),

        // ENDGAME
        ENDGAME_SENSOR1("Endgame Sensor 1"),
        ENDGAME_SENSOR2("Endgame Sensor 2"),
        ENDGAME_SENSOR3("Endgame Sensor 3"),
        ENDGAME_SENSOR4("Endgame Sensor 4"),
        ENDGAME_ENCODER("Endgame Encoder"),

        // DRIVE TRAIN
        LEFT_SPEED("Speed of left drivetrain"),
        RIGHT_SPEED("Speed of right drivetrain"),
        DRIVETRAIN_SHIFTED("Drivetrain shifted"),

        // MISCELLANEOUS
        CAMERA_STREAM("Camera stream"),
        DISTANCE_FROM_GOAL("Distance from goal"),
        PHOTON_READY("Photon Ready?"),

        AIMED("Is Aimed?"),
        PHOTON_YAW("Photon Angle");

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

        public T getData() {
            return m_data;
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

        @Override
        public boolean equals(Object obj) {
            MapData compareData = (MapData) obj;

            return compareData.m_dataContainer.getData().equals(m_dataContainer.getData());
        }
    }
}