package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;
import org.firstinspires.ftc.teamcode.Team9113.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Team9113.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Arrays;

@Config
@Autonomous(name = "NFSAuto", group = "Linear Opmode")
public class NFSAuto extends LinearOpMode {
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static final int HORIZON = 260; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, true);
        robot.initHardware(); //added
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
        UGContourRingPipeline.Config.setLowerOrange(new Scalar (0.0, 50.0, 0.0));
        UGContourRingPipeline.Config.setUpperOrange(new Scalar (255.0, 230.0, 110.0));
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN));
        Pose2d startPose = new Pose2d(-63, -44, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][8];
        traj[0][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-5, -57), Math.toRadians(0))
                .addTemporalMarker(.25, robot::intakeDown)
                .addTemporalMarker(.75, robot::wobbleDown)
                .addTemporalMarker(1.7, robot::openClaw)
                .addTemporalMarker(1.9, () -> {
                    robot.wobbleUp();
                    robot.flapLowerPosition();
                })
                .build();
        traj[0][1] = drive.trajectoryBuilder(traj[0][0].end().plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(-11))
                .addTemporalMarker(.2, () -> {
                    drive.flywheels.doMaxVelocity();
                })
                .build();
        traj[0][2] = drive.trajectoryBuilder(traj[0][1].end(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(1.5, () -> {
                    robot.wobbleDown();
                    robot.flapLowerPosition();
                })
                .splineTo(new Vector2d(-32, -24), Math.toRadians(-135))
                .build();
        traj[0][3] = drive.trajectoryBuilder(traj[0][2].end(), false)
                .splineTo(new Vector2d(-2, -48), Math.toRadians(-20))
                .addTemporalMarker(.9, robot::wobbleDown)
                .build();
        traj[0][4] = drive.trajectoryBuilder(traj[0][3].end(), true)
                .splineTo(new Vector2d(-12, -36), Math.toRadians(-45))
                .build();
        traj[0][5] = drive.trajectoryBuilder(traj[0][3].end().plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .splineTo(new Vector2d(12, -24), Math.toRadians(0))
                .build();
        traj[1][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .addTemporalMarker(.1, robot::flapLowerPosition)
                .addTemporalMarker(.25, robot::intakeDown)
                .splineTo(new Vector2d(22.5, -42), Math.toRadians(30))
                .addTemporalMarker(1.5, robot::wobbleDown)
                .addDisplacementMarker(() -> {
                    drive.flywheels.doMaxVelocity();
                })
                .build();
        traj[1][1] = drive.trajectoryBuilder(traj[1][0].end().plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(-11))
                .build();
        traj[1][2] = drive.trajectoryBuilder(traj[1][1].end(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(.1, () -> {
                    robot.wobbleDown();
                    robot.flap.setPosition(0.28);
                    robot.openClaw();
                })
                .splineTo(new Vector2d(-32, -24), Math.toRadians(-135))
                .build();
        traj[1][3] = drive.trajectoryBuilder(traj[1][2].end(), false)
                .splineTo(new Vector2d(-24, -55), Math.toRadians(-20))
                .splineTo(new Vector2d(19, -42), Math.toRadians(45))
                .addTemporalMarker(1.75, () -> {
                    robot.wobbleDown();
                })
                .addDisplacementMarker(() -> {
                    robot.startIntake();
                })
                .build();
        traj[1][4] = drive.trajectoryBuilder(traj[1][3].end(), true)
                .splineTo(new Vector2d(-24, -36), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    drive.flywheels.doMaxVelocity();
                })
                .build();
        traj[1][5] = drive.trajectoryBuilder(traj[1][4].end(), false)
                .splineTo(new Vector2d(-2, -36), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                    robot.shootDisc();
                })
                .build();
        traj[1][6] = drive.trajectoryBuilder(traj[1][4].end(), false)
                .splineTo(new Vector2d(12, -36), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> {
                    drive.flywheels.halt();
                })
                .build();
        traj[2][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .splineTo(new Vector2d(40, -55), Math.toRadians(0))
                .addTemporalMarker(2, robot::wobbleDown)
                .build();
        traj[2][1] = drive.trajectoryBuilder(traj[2][0].end(), true)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(169))
                .addTemporalMarker(.5, () -> {
                    robot.flapUpperPosition();
                    drive.flywheels.doMaxVelocity();
                })
                .build();
        traj[2][2] = drive.trajectoryBuilder(traj[2][1].end(), true)
                .splineTo(new Vector2d(-16, -36), Math.toRadians(180))
                .build();
        traj[2][3] = drive.trajectoryBuilder(traj[2][1].end())
                .lineTo(new Vector2d(-10, -36))
                .addDisplacementMarker(robot::startIntake)
                .build();
        traj[2][4] = drive.trajectoryBuilder(traj[2][3].end())
                .lineTo(new Vector2d(-34, -36))
                .addDisplacementMarker(() -> {
                    drive.flywheels.doMaxVelocity();
                })
                .build();
        traj[2][5] = drive.trajectoryBuilder(traj[2][4].end(), false)
                .splineTo(new Vector2d(-4.5, -36), Math.toRadians(0))
                .build();
        traj[2][6] = drive.trajectoryBuilder(traj[2][5].end(), true)
                .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                .build();
        traj[2][7] = drive.trajectoryBuilder(traj[2][6].end(), false)
                .splineTo(new Vector2d(-4.5, -36), Math.toRadians(0))
                .build();
        double startTime = System.currentTimeMillis();
        while (!isStarted() && camera.getFps() < 0) {
            telemetry.addData("Status", "Not Ready");
        }
        String height = pipeline.getHeight().toString();
        double initTime = System.currentTimeMillis() - startTime;
        double safezoneThreshold = 2000;
        double safezoneTimer = System.currentTimeMillis();
        while (!isStarted() && System.currentTimeMillis() <= safezoneTimer + safezoneThreshold) {
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
        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPose);
        switch (height) {
            case "ZERO":
                telemetry.addData("There are no rings", "");
                drive.followTrajectory(traj[0][0]);
                drive.turn(Math.toRadians(90));
                robot.sleep(100);
                drive.followTrajectory(traj[0][1]);
                robot.shootDisc();
                robot.sleep(300);
                drive.turn(Math.toRadians(7));
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                drive.turn(Math.toRadians(7));
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                drive.flywheels.halt();
                drive.followTrajectory(traj[0][2]);
                robot.closeClaw();
                robot.sleep(300);
                robot.wobbleUp();
                robot.sleep(750);
                drive.followTrajectory(traj[0][3]);
                robot.openClaw();
                robot.sleep(200);
                robot.wobbleUp();
                robot.sleep(750);
                robot.closeClaw();
                drive.turn(Math.toRadians(90));
                robot.sleep(100);
                drive.followTrajectory(traj[0][5]);
                break;
            case "ONE":
                telemetry.addData("There is one ring", "");
                drive.followTrajectory(traj[1][0]);
                robot.openClaw();
                robot.sleep(200);
                robot.wobbleUp();
                drive.turn(Math.toRadians(90));
                robot.sleep(100);
                drive.followTrajectory(traj[1][1]);
                robot.shootDisc();
                robot.sleep(500);
                drive.turn(Math.toRadians(7));
                robot.sleep(200);
                robot.shootDisc();
                robot.sleep(500);
                drive.turn(Math.toRadians(7));
                robot.sleep(200);
                robot.shootDisc();
                robot.sleep(200);
                drive.flywheels.halt();
                drive.followTrajectory(traj[1][2]);
                robot.closeClaw();
                robot.sleep(300);
                robot.wobbleUp();
                robot.sleep(750);
                drive.followTrajectory(traj[1][3]);
                robot.openClaw();
                robot.sleep(200);
                robot.wobbleUp();
                robot.sleep(750);
                robot.closeClaw();
                drive.followTrajectory(traj[1][4]);
                drive.followTrajectory(traj[1][5]);
                drive.followTrajectory(traj[1][6]);
                break;
            case "FOUR":
                telemetry.addData("There are four rings", "");
                drive.followTrajectory(traj[2][0]);
                robot.openClaw();
                robot.sleep(200);
                robot.wobbleUp();
                robot.sleep(100);
                drive.followTrajectory(traj[2][1]);
                robot.shootDisc();
                robot.sleep(300);
                drive.turn(Math.toRadians(7));
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                drive.turn(Math.toRadians(7));
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                drive.flywheels.halt();
              /*  drive.followTrajectory(traj[2][2]); //knock
                drive.followTrajectory(traj[2][3]); //backup
                drive.followTrajectory(traj[2][4]); //intake
                drive.followTrajectory(traj[2][5]); //shoot
                robot.shootDisc();
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                drive.flywheels.halt();
                drive.followTrajectory(traj[2][6]); //intake
                drive.followTrajectory(traj[2][7]); //shoot 1
                robot.shootDisc();
                robot.sleep(300);
                drive.flywheels.halt(); */
                break;
        }
        telemetry.update();
    }
}