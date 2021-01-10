package org.firstinspires.ftc.teamcode.NFS.Robot;

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

public class Vision {
    private HardwareMap hwMap;
    public static int camera_width = 480, camera_height = 360, horizon = 260, init_threshold = 2000;
    public static boolean using_webcam = true, debug = false;
    public static String webcam_name = "Webcam 1";
    private double camera_init_time;

    private OpenCvCamera camera;
    private Telemetry telemetry;
    private UGContourRingPipeline pipeline;
    StopWatch stopwatch;

    public Vision(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        stopwatch = new StopWatch();
        int cameraMonitorViewId = this
                .hwMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hwMap.appContext.getPackageName()
                );
        if (using_webcam) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hwMap.get(WebcamName.class, webcam_name), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, debug));
        UGContourRingPipeline.Config.setCAMERA_WIDTH(camera_width);
        UGContourRingPipeline.Config.setHORIZON(horizon);
        UGContourRingPipeline.Config.setLowerOrange(new Scalar(0.0, 50.0, 0.0));
        UGContourRingPipeline.Config.setUpperOrange(new Scalar(255.0, 230.0, 110.0));
        camera.openCameraDeviceAsync(() -> camera.startStreaming(camera_width, camera_height, OpenCvCameraRotation.UPSIDE_DOWN));

        stopwatch.start();
    }

    private boolean hasFPS() {
        return camera.getFps() < 0;
    }

    public UGContourRingPipeline.Height getHeight() {
        return pipeline.getHeight();
    }

    private boolean hasInitializationThresholdElapsed() {
        return stopwatch.getTime() >= init_threshold;
    }

    public boolean isReadyToRead() {
        camera_init_time = stopwatch.getTime();
        return hasFPS();
    }

    public boolean isReady() {
        return hasFPS() && hasInitializationThresholdElapsed();
    }

    public double cameraInitTime() {
        return camera_init_time;
    }

}
