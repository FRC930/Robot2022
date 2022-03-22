package frc.robot.utilities;

// ----- IMPORTS ----- \\

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.MalformedURLException;
import java.net.Socket;
import java.net.URI;
import java.net.URL;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.net.http.WebSocket.Listener;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Scanner;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.msgpack.jackson.dataformat.Tuple;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;
import frc.robot.utilities.ShuffleboardUtility.ShuffleBoardData;
import frc.robot.utilities.ShuffleboardUtility.ShuffleboardKeys;

/**
 * <h3>PhotonVisionUtility</h3>
 * 
 * PhotonVisionUtility holds the cameras that we use for our aiming.
 */
public class PhotonVisionUtility {
    // ----- CONSTANTS ----- \\

    private static final String PHOTON_URL_NO_PORT = "10.9.30.25";
    private static final String PHOTON_URL = PHOTON_URL_NO_PORT + ":5800";
    private static final String WEBSOCKET_RELATIVE_PATH = "/websocket";
    private static final String CONFIG_ZIP_RELATIVE_PATH = "/api/settings/photonvision_config.zip";
    private static final String PIPELINE_JSON_RELATIVE_PATH = "/cameras/mmal_service_16.1/pipelines";
    private static final String TEMPORARY_UNZIP_LOCATION = Filesystem.getOperatingDirectory().getAbsolutePath()
            + "/settings_unzipped";

    // ----- VARIABLES ----- \\

    private PhotonCamera hubTracking = new PhotonCamera("PiCamera");
    private PhotonCamera ballTracking = new PhotonCamera("CargoCamera");

    // Set up the object mapper that we will use for sending data to Photon
    private ObjectMapper objectMapper = new ObjectMapper(new MessagePackFactory());
    private HttpClient httpClient = HttpClient.newHttpClient();
    private WebSocket ws;

    private List<Tuple<String, Double>> exposureValues = new ArrayList<>();

    // ----- STATICS ----- \\

    private static PhotonVisionUtility instance;

    // ----- CONSTRUCTOR ----- \\

    /**
     * <h3>PhotonVisionUtility</h3>
     * 
     * This contstructs the photonvision and should only ever be called once by the
     * getInstance method
     */
    private PhotonVisionUtility() {
        // Set the banner to show the driver that photon isn't ready yet
        ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                ShuffleboardKeys.PHOTON_READY, new ShuffleBoardData<Boolean>(false));

        // Only initialize if we are not simulating the robot. This prevents waiting a
        // long time for photon to initialize in sim, because it will not be able to
        // locate the pi
        if (Robot.isReal()) {
            // Start a scheduled executor service to wait a bit before trying to initialize
            // photon
            ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
            executorService.schedule(() -> {
                // ------ WAIT FOR PHOTON TO COME ONLINE ------ \\

                // Initialize to false so that the loop runs at least once
                boolean isPhotonOnline = false;
                do {
                    // Check if photon is online using a helper method
                    isPhotonOnline = isReachable(PHOTON_URL_NO_PORT, 5800, 10000);
                } while (!isPhotonOnline);

                // ------ END WAIT FOR PHOTON TO COME ONLINE ------ \\

                // ------ PIPELINE RETREIVAL ------ \\

                try {
                    // We can use the buffered input stream to get a file over the network, in this
                    // case from photon
                    BufferedInputStream zipFile = new BufferedInputStream(
                            new URL("http://" + PHOTON_URL + CONFIG_ZIP_RELATIVE_PATH).openStream());

                    // Set up the object mapper from jackson to read json from the photon zip
                    ObjectMapper objectMapper = new ObjectMapper();

                    // We need to unzip the files to a temporary directory, so we can create a file
                    // object that points to this directory, in this case CWD/settings_unzipped
                    File destDir = new File(TEMPORARY_UNZIP_LOCATION);
                    // The buffer that will hold the zip file as we are unzipping it
                    byte[] buffer = new byte[1024];
                    // Zip input stream holds a zip file in the process of unzipping
                    ZipInputStream zis = new ZipInputStream(zipFile);
                    // Each part of the file (both files and directories) is a zip entry
                    ZipEntry zipEntry = zis.getNextEntry();
                    // We want to get all of the files and directories out of the zip file, so
                    // looping until we have no entries left will ensure that that goal is achieved
                    while (zipEntry != null) {
                        // Create a new file using the zip entry and the destination directory
                        File newFile = newFile(destDir, zipEntry);
                        // Check to see if the zip entry is a directory: if it is, we make sure that the
                        // file returned by the newFile method is a directory and attempt to create all
                        // intermediate directories
                        if (zipEntry.isDirectory()) {
                            if (!newFile.isDirectory() && !newFile.mkdirs()) {
                                // This will run if either the file returned by newFile is not a directory (even
                                // though the zip entry is) or if we fail to make the intermediate directories
                                // (indicating most likely an access error)
                                throw new IOException("Failed to create directory " + newFile);
                            }
                        } else {
                            // This checks to see that the parent file object to the "newFile" is a
                            // directory. Since all files reside in directories, this should always be the
                            // case. Like the previous couple of statements, we also attempt to create the
                            // parent directories with parent.mkdirs(). If this fails it likely indicates a
                            // filesystem access error
                            File parent = newFile.getParentFile();
                            if (!parent.isDirectory() && !parent.mkdirs()) {
                                // Either the zip structure is bad (having a file as child of another file) or
                                // we have an filesystem access error
                                throw new IOException("Failed to create directory " + parent);
                            }

                            // With all these checks in place, we can be assured that the file we now have
                            // is an actual file, and we can now write the data from the zip to this file
                            // location

                            // We use a file output stream to write the file to the specified location
                            FileOutputStream fos = new FileOutputStream(newFile);
                            // This variable will be used to store the length of the buffer that we read.
                            // When this length goes to zero, it indicates that the buffer is empty, meaning
                            // that there is no more data to write to the current file
                            int len;
                            while ((len = zis.read(buffer)) > 0) {
                                // Write the data using our buffer and length variable
                                fos.write(buffer, 0, len);
                            }
                            // Close the file to prevent resource leaks
                            fos.close();
                        }
                        // Get the next entry. If we have exhasuted all the fils in the zip, this
                        // function will return null, and the exit condition of the loop will trigger
                        zipEntry = zis.getNextEntry();
                    }
                    // Close the zip entry and zip file to prevent resource leaks
                    zis.closeEntry();
                    zis.close();

                    // Determine the directory where the pipelines are stored. This will be
                    // CWD/%path% where path includes the following: the folder where we unzipped
                    // the settings, "cameras," the name of the camera, and "pipelines."
                    // Photonvision stores its pipelines in this directory *for now*, but this could
                    // change in the future
                    File pipelinesDir = new File(TEMPORARY_UNZIP_LOCATION + PIPELINE_JSON_RELATIVE_PATH);
                    // This will list the files (and directories) in the pipelines folder. Photon
                    // doesn't store any other directories in the pipelines folder, so we don't have
                    // to check if the entries are files or directories
                    String[] pipelinesList = pipelinesDir.list();
                    // Iterate over the list of pipeline json files
                    for (String piplineFile : pipelinesList) {
                        // Append the name of the pipeline (plus a forward slash to indicate that the
                        // json is in a subdirectory) to the path of the directory determined above.
                        File currentPipeline = new File(pipelinesDir.getPath() + "/" + piplineFile);
                        // fileContents starts empty and will be filled by all the file's data
                        String fileContents = "";
                        // Initialize a scanner with the file for the current pipeline
                        Scanner sc = new Scanner(currentPipeline);
                        // Iterate over the entire file until the scanner's buffer is empty
                        while (sc.hasNextLine()) {
                            fileContents += sc.nextLine() + "\n";
                        }
                        // Close the scanner to prevent resource leaks
                        sc.close();

                        // Parse the json we get from the pipeline files
                        // This first statement will get a json node from the file
                        JsonNode jsonNode = objectMapper.readTree(fileContents);
                        // Photon stores its pipeline settings in a list for some strange reason, so we
                        // need to get the second item in the list
                        JsonNode settingsMap = jsonNode.get(1);
                        // Get the values we will use from the map of all the settings
                        String pipelineName = settingsMap.get("pipelineNickname").asText();
                        int pipelineIndex = Integer.parseInt(settingsMap.get("pipelineIndex").asText());
                        double exposure = Double.parseDouble(settingsMap.get("cameraExposure").asText());

                        // Add the options to the shuffleboard chooser. This will allow the driver to
                        // select different pipelines
                        ShuffleboardUtility.getInstance().addPipelineChooser(pipelineName, pipelineIndex);
                        // This will photon what exposure value to set when changing the pipeline over
                        // the websocket
                        PhotonVisionUtility.getInstance().addPipeline(pipelineName, exposure);
                    }
                } catch (MalformedURLException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                // ------ END PIPELINE RETREIVAL ------ \\
                // ------ SET UP WEBSOCKET ------ \\

                // Surround with try/catch to make sure we don't error out the thread
                try {
                    // The websocket we will use to send data to photon
                    // The address was obtained by opening dev tools with photon open. This
                    // shouldn't change in the near future, but if it does, a new address can be
                    // found by using dev tools
                    ws = httpClient.newWebSocketBuilder().buildAsync(
                            // This URI will point to the location found using dev tools
                            URI.create("ws://" + PHOTON_URL + WEBSOCKET_RELATIVE_PATH),
                            // Our websocket listener doesn't actually do anything, because we don't need to
                            // use the values from the websocket for aiming
                            new WebsocketListener())
                            // Wait for the future to complete so we get the websocket right away
                            .get();

                    // Set up a timeout waiting for the websocket
                    int counter = 0;
                    // If the websocket is null (which it shouldn't be) then we wait until the
                    // future completes
                    while (ws == null) {
                        // Sleep for 100 ms so as to not hog the roborio cpu
                        Thread.sleep(100);
                        // If we have been waiting for more than 10 seconds, throw a timeout error
                        if (counter++ > 100) {
                            throw new RuntimeException("Error connecting to photon: Timeout");
                        }
                    }

                    // Sleep for 2 seconds to ensure websocket is able to send messages
                    Thread.sleep(2000);

                    // Use our method to set the picamera's exposure
                    setPiCameraExposure();

                    // Set the LEDs to be off
                    hubTracking.setLED(VisionLEDMode.kOff);

                    // Tell shuffleboard that photon is up and ready
                    ShuffleboardUtility.getInstance().putToShuffleboard(ShuffleboardUtility.driverTab,
                            ShuffleboardKeys.PHOTON_READY, new ShuffleBoardData<Boolean>(true));
                } catch (Exception e) {
                    e.printStackTrace();
                    System.out.println("****** ERROR CONNECTING TO PHOTONVISION ******");
                }

                // ------ END SET UP WEBSOCKET ------ \\
            }, 500, TimeUnit.MILLISECONDS);
        }
    }

    // ----- SINGLETON GET ----- \\

    /**
     * <h3>getInstance</h3>
     * 
     * This is the accessor method for the singleton. This ensures that there is
     * only ever one instance of the PhotonVisionUtility
     * 
     * @return the instance of PhotonVisionUtility
     */
    public static PhotonVisionUtility getInstance() {
        if (instance == null) {
            return (instance = new PhotonVisionUtility());
        }
        return instance;
    }

    // ----- METHODS ----- \\

    /**
     * <h3>newFile</h3>
     * 
     * Initializes a new file with the passed zip entry. This is used to the
     * location of where to store the file
     * 
     * @param destinationDir the directory to store the file
     * @param zipEntry       the zip entry on which to operate
     * @return a file that will store the contents of the zip entry
     * @throws IOException if there is an error writing the file
     */
    private static File newFile(File destinationDir, ZipEntry zipEntry) throws IOException {
        // This uses one of the overloaded constructors available for File that will
        // take a parent and a child. This ensures cross-platform compatability
        File destFile = new File(destinationDir, zipEntry.getName());

        // Get the directory and file paths. Using canonical paths allows us to make
        // sure that we have a unique representation of where the file is stored
        String destDirPath = destinationDir.getCanonicalPath();
        String destFilePath = destFile.getCanonicalPath();

        // Make sure that the file is inside of the destination directory
        if (!destFilePath.startsWith(destDirPath + File.separator)) {
            throw new IOException("Entry is outside of the target dir: " + zipEntry.getName());
        }

        return destFile;
    }

    /**
     * <h3>isReachable</h3>
     * 
     * This method attempts to connect to a host over the passed port, returning
     * true if successful and false if it fails
     * 
     * @param addr          the remote address
     * @param openPort      the port used for connection
     * @param timeoutMillis a timeout in milliseconds
     * @return whether the server responded in time
     */
    private static boolean isReachable(String addr, int openPort, int timeoutMillis) {
        try (Socket soc = new Socket()) {
            soc.connect(new InetSocketAddress(addr, openPort), timeoutMillis);
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * <h3>setPiCameraExposure</h3>
     * 
     * This method sets the pi camera's exposure with the values we have for the
     * current pipeline
     */
    public void setPiCameraExposure() {
        try {
            // Make sure that we will be able to actually communicate with photon
            if (ws == null) {
                throw new RuntimeException("Websocket not initialized");
            }

            // Get the current pipeline index from shuffleboard. This will ensure that we
            // set the exposure correctly using the map as well.
            int currentPipeline = ShuffleboardUtility.getInstance().getSelectedPipelineChooser();

            // Set up the map to set the pipeline
            LinkedHashMap<String, Object> pipelineToggleMap = new LinkedHashMap<>();
            // Tell photon to set the pipeline to the selected index
            pipelineToggleMap.put("currentPipeline", currentPipeline);
            // Tell photon that we want to operate on the camera at index 0 (PiCamera)
            pipelineToggleMap.put("cameraIndex", 0);

            // Use the object mapper to convert the java LinkedHashMap to a bson byte array.
            // BSON stands for binary json, and is just a binary representation of data
            byte[] convertedMap = objectMapper.writeValueAsBytes(pipelineToggleMap);
            // Send the data over the websocket
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            // Sleep three milliseconds to ensure the data gets sent
            Thread.sleep(3);

            // Set up the map used for setting exposure
            LinkedHashMap<String, Object> exposureToggleMap = new LinkedHashMap<>();
            LinkedHashMap<String, Object> exposureValueMap = new LinkedHashMap<>();
            // Set the exposure to the target plus 0.1. This will simulate clicking the up
            // arrow on the web interface.
            exposureValueMap.put("cameraExposure", (double) (exposureValues.get(currentPipeline).second() + 0.1));
            exposureValueMap.put("cameraIndex", 0);
            // Put the exposure values (and camera index) into the exposure toggle map to
            // send to photon
            exposureToggleMap.put("changePipelineSetting", exposureValueMap);

            // Once again, convert the hash map to a binary json
            convertedMap = objectMapper.writeValueAsBytes(exposureToggleMap);
            // Send data over the websocket
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            // Wait to ensure we don't overload the websocket
            Thread.sleep(3);

            // Change the exposure to the target. This simulates pressing the down arrow on
            // the web interface
            exposureValueMap.put("cameraExposure", (double) (exposureValues.get(currentPipeline).second()));

            // Convert and send data again
            convertedMap = objectMapper.writeValueAsBytes(exposureToggleMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);
        } catch (RuntimeException e) {
            e.printStackTrace();
            System.out.println("****** PHOTON WEBSOCKET ERROR ******");
        } catch (JsonProcessingException e) {
            e.printStackTrace();
            System.out.println("****** ERROR SETTING PHOTON SETTINGS ******");
        } catch (InterruptedException e) {
            e.printStackTrace();
            System.out.println("****** ERROR SETTING PHOTON SETTINGS ******");
        }
    }

    /**
     * <h3>setPiCameraPipeline</h3>
     * 
     * This method will use the websocket that is initialized in the constructor to
     * send a pipeline change request to photon
     * 
     * @param pipeline the pipeline index to set
     */
    public void setPiCameraPipeline(int pipeline) {
        try {
            if (ws == null) {
                throw new RuntimeException("Websocket not initialized");
            }
            // Set up the map to set the pipeline
            LinkedHashMap<String, Object> pipelineSetMap = new LinkedHashMap<>();
            pipelineSetMap.put("currentPipeline", pipeline);
            pipelineSetMap.put("cameraIndex", 0);

            // Convert and send data to photon
            byte[] convertedMap = objectMapper.writeValueAsBytes(pipelineSetMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("****** ERROR SETTING PHOTON PIPELINE ******");
        }
    }

    /**
     * <h3>addPipeline</h3>
     * 
     * This method will add a pipeline to the list with a specified exposure value
     * and name
     * 
     * @param name     the pipeline name obtained from photon
     * @param exposure the exposure obtained from photon
     */
    public void addPipeline(String name, double exposure) {
        exposureValues.add(new Tuple<String, Double>(name, exposure));
    }

    /**
     * <h3>getBallTrackingCamera</h3>
     * 
     * Returns the PhotonCamera representing the cargo camera
     * 
     * @return a reference to the cargo camera
     */
    public PhotonCamera getBallTrackingCamera() {
        return ballTracking;
    }

    /**
     * <h3>getHubTrackingCamera</h3>
     * 
     * Returns the PhotonCamera representing the hub aiming camera
     * 
     * @return a reference to the hub camera
     */
    public PhotonCamera getHubTrackingCamera() {
        return hubTracking;
    }

    /**
     * <h3>WebsocketListener</h3>
     * 
     * This class represents a "dummy" listener. We don't want to do anything with
     * the targeting data that photon sends back
     */
    private class WebsocketListener implements Listener {
    }
}
