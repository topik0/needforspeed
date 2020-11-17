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

        Pose2d startPose = new Pose2d(-63, 46, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][4];
        traj[2][0] = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 56), Math.toRadians(0))
                .build();
//        traj[2][1] = drive.trajectoryBuilder(new Pose2d(0, 56), true)
//                .lineToLinearHeading(new Pose2d(0, 30, Math.toRadians(165)))
//                .build();
        traj[2][1] = drive.trajectoryBuilder(new Pose2d(0, 56))
                .strafeLeft(30)
                .build();
        traj[2][2] = drive.trajectoryBuilder(new Pose2d(0, 30))
                .splineTo(new Vector2d(42, 48), Math.toRadians(60))
                .build();
        traj[0][2] = drive.trajectoryBuilder(new Pose2d(0, 30))
                .splineTo(new Vector2d(0, 35), Math.toRadians(0))
                .build();
        // Goto one block wobble area
        traj[1][2] = drive.trajectoryBuilder(new Pose2d(0, 30))
                .splineTo(new Vector2d(25, 25), Math.toRadians(60))
                .build();
        traj[1][3] = drive.trajectoryBuilder(new Pose2d(25, 25))
                .back(15)
                .build();
        traj[2][3] = drive.trajectoryBuilder(new Pose2d(42, 48))
                .back(30)
                .build();


        String height = pipeline.getHeight().toString();
        while (!isStarted()) {
            height = pipeline.getHeight().toString();
        }

        switch (height) {
            case "ZERO":
                telemetry.addData("There are no rings", "");
                drive.followTrajectory(traj[2][0]);
                drive.followTrajectory(traj[2][1]);
                robot.flapUpperPosition();
                sleep(100);
                robot.startFlywheels();
                sleep(1500);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.stopFlywheels();
                sleep(100);
                drive.followTrajectory(traj[0][2]);
                robot.wobbleDown();
                sleep(300);
                robot.openClaw();
                sleep(200);
                robot.wobbleUp();
                sleep(200);
                break;
            case "ONE":
                telemetry.addData("There is one ring", "");
                drive.followTrajectory(traj[2][0]);
                drive.followTrajectory(traj[2][1]);
                robot.flapUpperPosition();
                sleep(100);
                robot.startFlywheels();
                sleep(1000);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.stopFlywheels();
                sleep(100);
                drive.followTrajectory(traj[1][2]);
                robot.wobbleDown();
                sleep(300);
                robot.openClaw();
                sleep(500);
                robot.wobbleUp();
                sleep(200);
                drive.followTrajectory(traj[1][3]);
                break;
            case "FOUR":
                telemetry.addData("There are four rings", "");
                drive.followTrajectory(traj[2][0]);
                drive.followTrajectory(traj[2][1]);
                robot.flapUpperPosition();
                sleep(100);
                robot.startFlywheels();
                sleep(1000);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.shootDisc();
                sleep(100);
                robot.stopFlywheels();
                sleep(100);
                drive.followTrajectory(traj[2][2]);
                robot.wobbleDown();
                sleep(300);
                robot.openClaw();
                sleep(500);
                robot.wobbleUp();
                sleep(200);
                drive.followTrajectory(traj[2][3]);
                break;
        }
        telemetry.update();
    }
}