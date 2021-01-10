package org.firstinspires.ftc.teamcode.NFS.Vision;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Vision;

@TeleOp(name = "ContorTest", group = "Linear Opmode")
public class ContorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, telemetry);
        while (!isStarted() && vision.isReadyToRead())
            telemetry.addData("Vision Status", "Not Ready");
        UGContourRingPipeline.Height height;
        while (!isStarted() && vision.isReady()) {
            telemetry.addData("Vision Status", "Not Ready");
            telemetry.addData("Camera Initialization Time: ", vision.cameraInitTime());
            telemetry.update();
        }
        while (opModeIsActive()) {
            height = vision.getHeight();
            telemetry.addData("Current count: ", height);
            telemetry.update();
        }
    }
}