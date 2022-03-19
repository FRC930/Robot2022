package frc.robot.utilities;

import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.net.http.WebSocket.Listener;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.msgpack.jackson.dataformat.Tuple;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import frc.robot.Robot;

/**
 * <h3>PhotonVisionUtility</h3>
 * 
 * PhotonVisionUtility holds the cameras that we use for our aiming.
 */
public class PhotonVisionUtility {
    private PhotonCamera hubTracking = new PhotonCamera("PiCamera");
    private PhotonCamera ballTracking = new PhotonCamera("CargoCamera");

    // Set up the object mapper that we will use for sending data to Photon
    private ObjectMapper objectMapper = new ObjectMapper(new MessagePackFactory());
    private HttpClient httpClient = HttpClient.newHttpClient();
    private WebSocket ws;

    private List<Tuple<String, Double>> exposureValues = new ArrayList<>();

    private static PhotonVisionUtility instance;

    private PhotonVisionUtility() {
        hubTracking.setLED(VisionLEDMode.kOff);

        if (Robot.isReal()) {
            ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();
            executorService.schedule(() -> {
                HttpURLConnection photonGetConnection = null;
                do {
                    try {
                        System.out.println("****** STARTED INITIALIZING PHOTON ******");
                        // Connect to the photonvision over the radio
                        photonGetConnection = (HttpURLConnection) new URL("http://10.9.30.25:5800/")
                                .openConnection();
                        // Set the connection timeout
                        photonGetConnection.setConnectTimeout(100);
                        // Try reconnecting until we either get an error or get a response
                        while (photonGetConnection.getResponseCode() != 200) {
                            photonGetConnection.connect();
                        }
                        System.out.println("****** FINISHED INITIALIZING PHOTON ******");
                    } catch (Exception e) {
                        System.out.println("****** ERROR FINDING PHOTONVISION ******");
                    }
                } while (photonGetConnection == null);

                try {
                    // The websocket we will use to broadcast data to photon
                    ws = httpClient.newWebSocketBuilder().buildAsync(URI.create("ws://10.9.30.25:5800/websocket"),
                            new WebsocketListener()).get();

                    // Set the exposure for the pi camera
                    executorService.schedule(this::setPiCameraExposure, 1000, TimeUnit.MILLISECONDS);
                } catch (Exception e) {
                    System.out.println("****** ERROR CONNECTING TO PHOTONVISION ******");
                }
            }, 2000, TimeUnit.MILLISECONDS);
        }
    }

    public static PhotonVisionUtility getInstance() {
        if (instance == null) {
            return (instance = new PhotonVisionUtility());
        }
        return instance;
    }

    public void setPiCameraExposure() {
        try {
            if (ws == null) {
                throw new RuntimeException("Websocket not initialized");
            }
            System.out.println("****** STARTED SETTING PHOTON SETTINGS ******");

            int currentPipeline = ShuffleboardUtility.getInstance().getSelectedPipelineChooser();

            // Set up the driver mode toggle map
            LinkedHashMap<String, Object> pipelineToggleMap = new LinkedHashMap<>();
            // Tell photon to turn off driver mode
            pipelineToggleMap.put("currentPipeline", currentPipeline);
            // Tell photon that we want to operate on the camera at index 1 (PiCamera)
            pipelineToggleMap.put("cameraIndex", 0);

            // Set up the binary data we will use to send data to photon
            byte[] convertedMap = objectMapper.writeValueAsBytes(pipelineToggleMap);
            // Send the data over the websocket
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            Thread.sleep(3);

            // Set up the map used for setting exposure
            LinkedHashMap<String, Object> exposureToggleMap = new LinkedHashMap<>();
            LinkedHashMap<String, Object> exposureValueMap = new LinkedHashMap<>();
            // Set the exposure to the target plus 0.1
            exposureValueMap.put("cameraExposure", (double) (exposureValues.get(currentPipeline).second() + 0.1));
            exposureValueMap.put("cameraIndex", 0);
            exposureToggleMap.put("changePipelineSetting", exposureValueMap);

            // Convert the map to a binary json
            convertedMap = objectMapper.writeValueAsBytes(exposureToggleMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            Thread.sleep(3);

            // Change the exposure to the target
            exposureValueMap.put("cameraExposure", (double) (exposureValues.get(currentPipeline).second()));

            // Convert and send data again
            convertedMap = objectMapper.writeValueAsBytes(exposureToggleMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            System.out.println("****** SET PHOTON SETTINGS ******");
        } catch (RuntimeException e) {
            System.out.println("****** PHOTON WEBSOCKET WAS NOT INITIALIZED ******");
        } catch (JsonProcessingException e) {
            System.out.println("****** ERROR SETTING PHOTON SETTINGS ******");
        } catch (InterruptedException e) {
            System.out.println("****** ERROR SETTING PHOTON SETTINGS ******");
        }
    }

    public void setPiCamerPipeline(int pipeline) {
        try {
            if (ws == null) {
                throw new RuntimeException("Websocket not initialized");
            }
            // Set up the map to set the pipeline
            LinkedHashMap<String, Object> pipelineSetMap = new LinkedHashMap<>();
            pipelineSetMap.put("currentPipeline", pipeline);
            pipelineSetMap.put("cameraIndex", 1);

            // Convert and send data to photon
            byte[] convertedMap = objectMapper.writeValueAsBytes(pipelineSetMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);
        } catch (Exception e) {
            System.out.println("****** ERROR SETTING PHOTON PIPELINE ******");
        }
    }

    public void addPipeline(String name, double exposure) {
        exposureValues.add(new Tuple<String, Double>(name, exposure));
    }

    public PhotonCamera getBallTrackingCamera() {
        return ballTracking;
    }

    public PhotonCamera getHubTrackingCamera() {
        return hubTracking;
    }

    private class WebsocketListener implements Listener {
    }
}
