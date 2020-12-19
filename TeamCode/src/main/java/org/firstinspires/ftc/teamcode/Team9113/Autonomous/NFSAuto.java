package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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

@Autonomous(name = "NFSAuto", group = "Linear Opmode")
public class NFSAuto extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(-63, -44, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][10];
        //
        traj[0][0] = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(0, -60), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                    robot.openClaw();
                    robot.wobbleUp();
                    robot.flapLowerPosition();
                    robot.startFlywheels();
                    robot.delay(150);
                })
                .build();
        traj[0][1] = drive.trajectoryBuilder(traj[0][0].end(), true)
                .splineTo(new Vector2d(0, -12), Math.toRadians(185))
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    drive.turnAsync(Math.toRadians(-5));
                    robot.delay(75);
                })
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    drive.turnAsync(Math.toRadians(-5));
                    robot.delay(75);
                })
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    robot.delay(75);
                    robot.stopFlywheels();
                })
                .build();
        traj[0][2] = drive.trajectoryBuilder(traj[0][1].end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(-45))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                })
                .splineTo(new Vector2d(-30, -14), Math.toRadians(-315))
                .addDisplacementMarker(() -> {
                    robot.closeClaw();
                    robot.delay(75);
                    robot.wobbleUp();
                    robot.delay(150);
                })
                .build();
        traj[0][3] = drive.trajectoryBuilder(traj[0][2].end(), true)
                .splineTo(new Vector2d(0, -48), Math.toRadians(160))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                    robot.openClaw();
                    robot.wobbleUp();
                    robot.delay(150);
                    robot.closeClaw();
                })
                .build();
        traj[0][4] = drive.trajectoryBuilder(traj[0][3].end())
                .splineTo(new Vector2d(-12, -36), Math.toRadians(135))
                .build();
        traj[0][5] = drive.trajectoryBuilder(traj[0][4].end(), true)
                .splineTo(new Vector2d(12, -24), Math.toRadians(0))
                .build();
        traj[1][0] = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(200))
                .splineTo(new Vector2d(0, -12), Math.toRadians(185))
                .addDisplacementMarker(() -> {
                    robot.flapLowerPosition();
                    robot.shootDisc();
                    drive.turnAsync(Math.toRadians(-5));
                    robot.delay(75);
                })
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    drive.turnAsync(Math.toRadians(-5));
                    robot.delay(75);
                })
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    robot.delay(75);
                    robot.stopFlywheels();
                })
                .build();
        traj[1][1] = drive.trajectoryBuilder(traj[1][0].end(), true)
                .splineTo(new Vector2d(34, -24), Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                    robot.openClaw();
                    robot.wobbleUp();
                })
                .build();
        traj[1][2] = drive.trajectoryBuilder(traj[1][1].end())
                .splineTo(new Vector2d(24, -16), Math.toRadians(180))
                .addDisplacementMarker(robot::wobbleDown)
                .build();
        traj[1][3] = drive.trajectoryBuilder(traj[1][2].end(), true)
                .splineTo(new Vector2d(-33, -17), 0)
                .addDisplacementMarker(() -> {
                    robot.closeClaw();
                    robot.delay(75);
                    robot.wobbleUp();
                    robot.delay(150);
                })
                .splineTo(new Vector2d(26, -16), Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                    robot.openClaw();
                    robot.wobbleUp();
                    robot.delay(150);
                    robot.closeClaw();
                    robot.startIntake();
                    robot.flapUpperPosition();
                })
                .build();
        traj[1][4] = drive.trajectoryBuilder(traj[1][3].end())
                .splineTo(new Vector2d(-19, -36), Math.toRadians(180))
                .addDisplacementMarker(robot::startFlywheels)
                .build();
        traj[1][5] = drive.trajectoryBuilder(traj[1][4].end(), true)
                .splineTo(new Vector2d(0, -36), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                    robot.shootDisc();
                    robot.delay(50);
                })
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
                .build();
        traj[2][0] = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(200))
                .splineTo(new Vector2d(0, -12), Math.toRadians(185))
                .addDisplacementMarker(() -> {
                    robot.flapLowerPosition();
                    robot.shootDisc();
                    drive.turnAsync(Math.toRadians(-5));
                    robot.delay(75);
                })
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    drive.turnAsync(Math.toRadians(-5));
                    robot.delay(75);
                })
                .addDisplacementMarker(() -> {
                    robot.shootDisc();
                    robot.delay(75);
                    robot.stopFlywheels();
                })
                .build();
        traj[2][1] = drive.trajectoryBuilder(traj[2][0].end(), true)
                .splineTo(new Vector2d(58, -44), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                    robot.openClaw();
                    robot.wobbleUp();
                })
                .build();
        traj[2][2] = drive.trajectoryBuilder(traj[2][1].end())
                .splineTo(new Vector2d(58, -36), Math.toRadians(90))
                .build();
        traj[2][3] = drive.trajectoryBuilder(traj[2][2].end(), true)
                .splineTo(new Vector2d(-33, -17), 0)
                .addDisplacementMarker(() -> {
                    robot.closeClaw();
                    robot.delay(75);
                    robot.wobbleUp();
                    robot.delay(150);
                })
                .splineTo(new Vector2d(-36, 0), Math.toRadians(225))
                .splineTo(new Vector2d(50, -40), Math.toRadians(135))
                .addDisplacementMarker(() -> {
                    robot.wobbleDown();
                    robot.delay(150);
                    robot.openClaw();
                    robot.wobbleUp();
                    robot.closeClaw();
                })
                .build();
        traj[2][4] = drive.trajectoryBuilder(traj[2][3].end())
                .splineTo(new Vector2d(-12, -36), Math.toRadians(180))
                .build();
        traj[2][5] = drive.trajectoryBuilder(traj[2][4].end(), true)
                .lineTo(new Vector2d(-6, -36))
                .addDisplacementMarker(() -> {
                    robot.startIntake();
                    robot.delay(150);
                })
                .build();
        traj[2][6] = drive.trajectoryBuilder(traj[2][5].end())
                .lineTo(new Vector2d(-24, -36))
                .build();
        traj[2][7] = drive.trajectoryBuilder(traj[2][6].end(), true)
                .addDisplacementMarker(robot::startFlywheels)
                .splineTo(new Vector2d(0, -36), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                    robot.flapUpperPosition();
                    robot.shootDisc();
                    robot.delay(150);
                    robot.shootDisc();
                    robot.delay(150);
                    robot.shootDisc();
                    robot.stopFlywheels();
                    robot.startIntake();
                })
                .build();
        traj[2][8] = drive.trajectoryBuilder(traj[2][7].end())
                .splineTo(new Vector2d(-24, -36), Math.toRadians(180))
                .addDisplacementMarker(robot::startFlywheels)
                .build();
        traj[2][9] = drive.trajectoryBuilder(traj[2][8].end(), true)
                .splineTo(new Vector2d(0, -36), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                    robot.shootDisc();
                    robot.delay(100);
                })
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
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
                for(int i = 0; i < 5; i++)
                    drive.followTrajectory(traj[0][i]);
                break;
            case "ONE":
                telemetry.addData("There is one ring", "");
                for(int i = 0; i < 5; i++)
                    drive.followTrajectory(traj[1][i]);
                break;
            case "FOUR":
                telemetry.addData("There are four rings", "");
                for(int i = 0; i < 9; i++)
                    drive.followTrajectory(traj[2][i]);
                break;
        }
        telemetry.update();
    }
}