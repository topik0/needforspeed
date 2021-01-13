package org.firstinspires.ftc.teamcode.NFS.Vision;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is an OP Mode that runs the robot vision to detect the number of rings in a stack
 */
@TeleOp(name = "Vision Test", group = "Linear Opmode")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, telemetry);
        while (!isStarted() && vision.isNotReadyToRead()) {
            telemetry.addData("Vision Status", "Not Ready");
            telemetry.update();
        }
        UGContourRingPipeline.Height height;
        while (!isStarted() && vision.isNotReady()) {
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