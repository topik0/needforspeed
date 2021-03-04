package org.firstinspires.ftc.teamcode.NFS.Vision;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.time.StopWatch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is an OP Mode that runs the robot vision to detect the number of rings in a stack
 */
public class Vision {
    /**
     * Camera Dimensions
     */
    public static int camera_width = 640, camera_height = 360, horizon = 130, init_threshold = 2000;
    public static boolean using_webcam = true, debug = false;
    /**
     * The name of the webcam in the robot configuration
     */
    public static String webcam_name = "Webcam 1";
    private double camera_init_time;

    private final OpenCvWebcam webcam;
    private final UGContourRingPipeline pipeline;
    private final StopWatch stopwatch;

    public Vision(HardwareMap hwMap, Telemetry telemetry) {
        stopwatch = new StopWatch();
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"));
        webcam.setPipeline(pipeline = new UGContourRingPipeline(telemetry, debug));
        UGContourRingPipeline.Config.setCAMERA_WIDTH(camera_width);
        UGContourRingPipeline.Config.setHORIZON(horizon);
        UGContourRingPipeline.Config.setLowerOrange(new Scalar(0.0, 50.0, 0.0));
        UGContourRingPipeline.Config.setUpperOrange(new Scalar(255.0, 230.0, 110.0));
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(camera_width, camera_height, OpenCvCameraRotation.UPSIDE_DOWN));

        stopwatch.start();
    }

    /**
     * Returns true if the camera is running
     *
     * @return true if the camera has a FPS higher than zero
     */
    private boolean hasFPS() {
        return webcam.getFps() > 0;
    }

    /**
     * Gets the height of the ring stack as detected by the camera
     *
     * @return the height of the ring stack
     */
    public UGContourRingPipeline.Height getHeight() {
        return pipeline.getHeight();
    }

    /**
     * Checks if the initialization threshold for the camera has elapsed
     *
     * @return returns true if the initialization threshold has elapsed
     */
    private boolean hasInitializationThresholdElapsed() {
        return stopwatch.getTime() >= init_threshold;
    }

    /**
     * Checks if the camera is ready to read the ring stack
     *
     * @return true if the camera is ready to read
     */
    public boolean isNotReadyToRead() {
        camera_init_time = stopwatch.getTime();
        return !hasFPS();
    }

    /**
     * Checks if the camera is ready to read the ring stack after the initial detections
     *
     * @return true if the camera is ready for normal vision use
     */
    public boolean isNotReady() {
        return !(hasFPS() && hasInitializationThresholdElapsed());
    }

    /**
     * Gets the time the camera took to initialize
     *
     * @return the initialization time in milliseconds
     */
    public double cameraInitTime() {
        return camera_init_time;
    }



    public void stopStreaming(){
        webcam.closeCameraDevice();
    }

}