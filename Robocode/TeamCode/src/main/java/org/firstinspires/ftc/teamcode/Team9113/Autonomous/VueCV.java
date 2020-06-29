package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Team9113.eocv.NaiveRectangleSamplingSkystoneDetectionPipelineTwo;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class VueCV {
    private OpenCvCamera phoneCam;
    private NaiveRectangleSamplingSkystoneDetectionPipelineTwo pipeline;
    private int blockPos = -1;
    public VueCV(@NotNull HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        pipeline = new NaiveRectangleSamplingSkystoneDetectionPipelineTwo();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
    }
    public int getBlockPosition(){
        return pipeline.getDetectedSkystonePosition();
    }
    public boolean block(int is){
        return is == blockPos;
    }
    public void updatePosition(){
        blockPos = pipeline.getDetectedSkystonePosition();
    }
    public void terminate(){
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
        blockPos = -1;
    }
}
