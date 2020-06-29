package org.firstinspires.ftc.teamcode.Team9113.eocv;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp(name = "EasyOpenCVTest", group = "testing")
public class eocvTest extends OpMode
{
    private OpenCvCamera camera;

    @Override
    public void init()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        camera.openCameraDevice();

        camera.setPipeline(new NaivePointSampleSkystoneDetectionPipeline());

        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop()
    {
        telemetry.addData("Frame Count", camera.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", camera.getFps()));
        telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
        telemetry.update();
    }
}
