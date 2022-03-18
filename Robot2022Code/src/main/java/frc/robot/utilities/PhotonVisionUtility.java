package frc.robot.utilities;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.net.http.WebSocket.Listener;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Scanner;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import com.fasterxml.jackson.databind.ObjectMapper;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.Filesystem;

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

    HashMap<Integer, Double> exposureMap = new HashMap<>();

    // Change this value to change the exposure we send to photonvision
    private static double PICAMERA_EXPOSURE = 1.4;

    private static PhotonVisionUtility instance;

    private PhotonVisionUtility() {
        hubTracking.setLED(VisionLEDMode.kOff);

        HttpURLConnection photonGetConnection = null;
        do {
            try {
                // Connect to the photonvision over the radio
                photonGetConnection = (HttpURLConnection) new URL("http://10.9.30.25:5800/")
                        .openConnection();
                // Set the connection timeout
                photonGetConnection.setConnectTimeout(100);
                // Try reconnecting until we either get an error or get a response
                while (photonGetConnection.getResponseCode() != 200) {
                    photonGetConnection.connect();
                }
            } catch (Exception e) {
                System.out.println("****** ERROR FINDING PHOTONVISION ******");
            }
        } while (photonGetConnection == null);

        try {
            // The websocket we will use to broadcast data to photon
            ws = httpClient.newWebSocketBuilder().buildAsync(URI.create("ws://10.9.30.25:5800/websocket"),
                    new WebsocketListener()).get();

            // Set the exposure for the pi camera
            setPiCameraExposure();
        } catch (Exception e) {
            System.out.println("****** ERROR CONNECTING TO PHOTONVISION ******");
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
            System.out.println("****** STARTED SETTING PHOTON SETTINGS ******");

            // Set up the driver mode toggle map
            LinkedHashMap<String, Object> driverModeToggleMap = new LinkedHashMap<>();
            // Tell photon to turn off driver mode
            driverModeToggleMap.put("currentPipeline", 0);
            // Tell photon that we want to operate on the camera at index 1
            driverModeToggleMap.put("cameraIndex", 1);

            // Set up the binary data we will use to send data to photon
            byte[] convertedMap = objectMapper.writeValueAsBytes(driverModeToggleMap);
            // Send the data over the websocket
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            Thread.sleep(3);

            // Set up the map used for setting exposure
            LinkedHashMap<String, Object> exposureToggleMap = new LinkedHashMap<>();
            LinkedHashMap<String, Object> exposureValueMap = new LinkedHashMap<>();
            // Set the exposure to the target plus 0.1
            exposureValueMap.put("cameraExposure", PICAMERA_EXPOSURE + 0.1);
            exposureValueMap.put("cameraIndex", 1);
            exposureToggleMap.put("changePipelineSetting", exposureValueMap);

            // Convert the map to a binary json
            convertedMap = objectMapper.writeValueAsBytes(exposureToggleMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            Thread.sleep(3);

            // Change the exposure to the target
            exposureValueMap.put("cameraExposure", PICAMERA_EXPOSURE);

            // Convert and send data again
            convertedMap = objectMapper.writeValueAsBytes(exposureToggleMap);
            ws.sendBinary(ByteBuffer.wrap(convertedMap), true);

            System.out.println("****** SET PHOTON SETTINGS ******");
        } catch (Exception e) {
            System.out.println("****** ERROR SETTING PHOTON SETTINGS ******");
        }
    }

    public void setPiCamerPipeline(int pipeline) {
        try {
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

    public PhotonCamera getBallTrackingCamera() {
        return ballTracking;
    }

    public PhotonCamera getHubTrackingCamera() {
        return hubTracking;
    }

    private class WebsocketListener implements Listener {
    }
}
