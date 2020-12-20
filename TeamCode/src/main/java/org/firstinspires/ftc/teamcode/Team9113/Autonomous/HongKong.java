package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Team9113.Robot;
import org.firstinspires.ftc.teamcode.Team9113.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "HongKong", group = "Linear Opmode")
public class HongKong extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution


    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        robot.startPositions(true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        UGContourRingPipeline pipeline;
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        Pose2d startPose = new Pose2d(-63, 45, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][5];
        traj[0][0] = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-9, 58), Math.toRadians(15))
                .build();
        traj[0][1] = drive.trajectoryBuilder(new Pose2d(-9, 58), true)
                .splineTo(new Vector2d(0, 36), Math.toRadians(-9.5))
                .build();
        traj[0][2] = drive.trajectoryBuilder(new Pose2d(0, 36), true)
                .splineTo(new Vector2d(-37, 15.5), Math.toRadians(180))
                .build();
        traj[0][3] = drive.trajectoryBuilder(new Pose2d(-37, 15.5),true)
                .splineTo(new Vector2d(0, 12), 0)
                .splineTo(new Vector2d(10, 44), Math.toRadians(90))
                .build();
        traj[0][4] = drive.trajectoryBuilder(new Pose2d(10, 44))
                .splineTo(new Vector2d(12, 24), Math.toRadians(180))
                .build();
        traj[1][0] = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-24, 56), 0)
                .splineTo(new Vector2d(0, 36), Math.toRadians(-9.5))
                .build();
        traj[1][1] = drive.trajectoryBuilder(new Pose2d(0, 36), true)
                .splineTo(new Vector2d(20, 38), Math.toRadians(20))
                .build();
        traj[1][2] = drive.trajectoryBuilder(new Pose2d(20, 38), true)
                .splineTo(new Vector2d(12, 10), Math.toRadians(180))
                .splineTo(new Vector2d(-39, 10), Math.toRadians(157.5))
                .build();
        traj[1][3] = drive.trajectoryBuilder(new Pose2d(-39, 10), true)
                .splineTo(new Vector2d(0, 18), 0)
                .splineTo(new Vector2d(16, 25), Math.toRadians(45))
                .build();
        traj[1][4] = drive.trajectoryBuilder(new Pose2d(16, 25))
                .splineTo(new Vector2d(12.0, 24.0), Math.toRadians(180))
                .build();
        traj[2][0] = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-24, 56), 0)
                .splineTo(new Vector2d(0, 36), Math.toRadians(-9.5))
                .build();
        traj[2][1] = drive.trajectoryBuilder(new Pose2d(0, 36), true)
                .splineTo(new Vector2d(22, 26), Math.toRadians(12))
                .splineTo(new Vector2d(48, 52), Math.toRadians(45))
                .build();
        traj[2][2] = drive.trajectoryBuilder(new Pose2d(48, 52), true)
                .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                .splineTo(new Vector2d(-40, 12.5), Math.toRadians(157.5))
                .build();
        traj[2][3] = drive.trajectoryBuilder(new Pose2d(-40, 12.5), true)
                .splineTo(new Vector2d(-24, 12), 0)
                .splineTo(new Vector2d(40, 54), Math.toRadians(45))
                .build();
        traj[2][4] = drive.trajectoryBuilder(new Pose2d(40, 54), true)
                .addTemporalMarker(.75, robot::closeClaw)
                .splineTo(new Vector2d(12, 24), 0)
                .build();
        double startTime = System.currentTimeMillis();
        while (camera.getFps() < 0) {
            telemetry.addData("Status", "Not Ready");
        }
        String height = pipeline.getHeight().toString();
        double initTime = System.currentTimeMillis() - startTime;
        double safezoneThreshold = 2000;
        double safezoneTimer = System.currentTimeMillis();
        while (System.currentTimeMillis() <= safezoneTimer + safezoneThreshold) {
            height = pipeline.getHeight().toString();
            telemetry.addData("Status", "Not Ready");
            telemetry.addData("Camera Initialization Time: ", initTime);
            telemetry.update();
        }
        while (!isStarted()) {
            height = pipeline.getHeight().toString();
            telemetry.addData("Status", "Ready");
            telemetry.addData("Ring count: ", height);
            telemetry.addData("Camera Initialization Time: ", initTime);
            telemetry.update();
        }

        switch (height) {
            case "ZERO":
                telemetry.addData("There are no rings", "");
                drive.followTrajectory(traj[0][0]);
                robot.wobbleDown();
                robot.intakeDown();
                sleep(600);
                robot.openClaw();
                sleep(200);
                robot.wobbleUp();
                sleep(200);
                drive.followTrajectory(traj[0][1]);
                robot.flapUpperPosition();
                sleep(100);
                robot.flywheelFast();
                sleep(1000);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                robot.stopFlywheels();
                robot.wobbleDown();
                drive.followTrajectory(traj[0][2]);
                robot.closeClaw();
                sleep(400);
                robot.wobbleUp();
                sleep(500);
                drive.followTrajectory(traj[0][3]);
                robot.wobbleDown();
                sleep(500);
                robot.openClaw();
                sleep(150);
                robot.wobbleUp();
                sleep(200);
                robot.closeClaw();
                drive.followTrajectory(traj[0][4]);
                break;
            case "ONE":
                telemetry.addData("There is one ring", "");
                drive.followTrajectory(traj[1][0]);
                robot.flapUpperPosition();
                sleep(100);
                robot.flywheelFast();
                sleep(1000);
                robot.shootDisc();
                sleep(120);
                robot.shootDisc();
                sleep(120);
                robot.shootDisc();
                sleep(160);
                robot.shootDisc();
                robot.stopFlywheels();
                drive.followTrajectory(traj[1][1]);
                robot.wobbleDown();
                sleep(500);
                robot.openClaw();
                sleep(100);
                drive.followTrajectory(traj[1][2]);
                robot.closeClaw();
                sleep(400);
                robot.wobbleUp();
                sleep(500);
                drive.followTrajectory(traj[1][3]);
                robot.wobbleDown();
                sleep(350);
                robot.openClaw();
                sleep(100);
                robot.wobbleUp();
                sleep(200);
                robot.closeClaw();
                drive.followTrajectory(traj[1][4]);
                break;
            case "FOUR":
                telemetry.addData("There are four rings", "");
                drive.followTrajectory(traj[2][0]);
                robot.flapUpperPosition();
                sleep(100);
                robot.flywheelFast();
                sleep(1000);
                robot.shootDisc();
                sleep(120);
                robot.shootDisc();
                sleep(120);
                robot.shootDisc();
                sleep(120);
                robot.shootDisc();
                robot.stopFlywheels();
                drive.followTrajectory(traj[2][1]);
                robot.wobbleDown();
                sleep(300);
                robot.openClaw();
                sleep(100);
                drive.followTrajectory(traj[2][2]);
                robot.closeClaw();
                sleep(400);
                robot.wobbleUp();
                sleep(500);
                drive.followTrajectory(traj[2][3]);
                robot.wobbleDown();
                sleep(350);
                robot.openClaw();
                sleep(100);
                robot.wobbleUp();
                sleep(200);
                drive.followTrajectory(traj[2][4]);
                break;
        }
        telemetry.update();
    }
}