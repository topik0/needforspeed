package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;
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
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        Pose2d startPose = new Pose2d(-63, -44, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][8];
        traj[0][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-5, -57), Math.toRadians(0))
                .addTemporalMarker(.75, robot::wobbleDown)
                .addTemporalMarker(1.7, robot::openClaw)
                .addTemporalMarker(1.9, () -> {
                    robot.wobbleUp();
                    robot.flapLowerPosition();
                })
                .build();
        traj[0][1] = drive.trajectoryBuilder(traj[0][0].end(), false)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(-11))
                .addTemporalMarker(.2, () -> {
                    //SampleMecanumDrive.flyWheelVelocity = 1550;
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
                .addTemporalMarker(.75, robot::wobbleDown)
                .build();
        traj[0][4] = drive.trajectoryBuilder(traj[0][3].end(), true)
                .splineTo(new Vector2d(-12, -36), Math.toRadians(-45))
                .build();
        traj[0][5] = drive.trajectoryBuilder(traj[0][3].end(), false)
                .splineTo(new Vector2d(12, -24), Math.toRadians(0))
                .build();
        traj[1][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .addTemporalMarker(.1, robot::flapLowerPosition)
                .splineTo(new Vector2d(24, -31), Math.toRadians(0))
                .addTemporalMarker(2, robot::wobbleDown)
                .addDisplacementMarker(() -> {
                    //SampleMecanumDrive.flyWheelVelocity = 1550;
                })
                .build();
        traj[1][1] = drive.trajectoryBuilder(traj[1][0].end(), true)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(175))
                .build();
        traj[1][2] = drive.trajectoryBuilder(traj[1][1].end(), true)
                .splineTo(new Vector2d(-24, -36), Math.toRadians(-135))
                .addTemporalMarker(.1, () -> {
                    robot.startIntake();
                    robot.flapLowerPosition();
                })
                .build();
        traj[1][3] = drive.trajectoryBuilder(traj[1][2].end(), false)
                .splineTo(new Vector2d(-4.5, -36), Math.toRadians(-5))
                .addTemporalMarker(.1, () -> {
                    //SampleMecanumDrive.flyWheelVelocity = 1550;
                    robot.flapUpperPosition();
                })
                .addTemporalMarker(1, robot::stopIntake)
                .build();
        traj[1][4] = drive.trajectoryBuilder(traj[1][3].end(), true)
                .splineTo(new Vector2d(-19, -36), Math.toRadians(0))
                // .addDisplacementMarker(robot::startFlywheels)
                .build();
        traj[1][5] = drive.trajectoryBuilder(traj[1][4].end(), false)
                .splineTo(new Vector2d(0, -36), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.stopIntake();
                    robot.shootDisc();
                    robot.delay(50);
                })
                .splineTo(new Vector2d(12, -36), Math.toRadians(0))
                .build();
        traj[2][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .splineTo(new Vector2d(40, -55), Math.toRadians(0))
                .addTemporalMarker(2, robot::wobbleDown)
                .build();
        traj[2][1] = drive.trajectoryBuilder(traj[2][0].end(), true)
                .splineTo(new Vector2d(-4.5, -36), Math.toRadians(180))
                .addTemporalMarker(.5, () -> {
                    robot.flapUpperPosition();
                    //SampleMecanumDrive.flyWheelVelocity = 1550;
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
                    //SampleMecanumDrive.flyWheelVelocity = 1550;
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
                //SampleMecanumDrive.flyWheelVelocity = 0;
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
                drive.followTrajectory(traj[0][5]);
                break;
            case "ONE":
                telemetry.addData("There is one ring", "");
                drive.followTrajectory(traj[1][0]);
                robot.openClaw();
                robot.sleep(200);
                robot.wobbleUp();
                drive.followTrajectory(traj[1][1]);
                robot.shootDisc();
                robot.sleep(200);
                drive.turn(Math.toRadians(-10));
                robot.sleep(200);
                robot.shootDisc();
                robot.sleep(200);
                drive.turn(Math.toRadians(-5));
                robot.sleep(200);
                robot.shootDisc();
                robot.sleep(200);
                //SampleMecanumDrive.flyWheelVelocity = 0;
                drive.followTrajectory(traj[1][2]);
                drive.followTrajectory(traj[1][3]);
                robot.shootDisc();
                robot.sleep(200);
                //SampleMecanumDrive.flyWheelVelocity = 0;
                break;
            case "FOUR":
                telemetry.addData("There are four rings", "");
                //for(int i = 0; i < 9; i++)
                drive.followTrajectory(traj[2][0]);
                robot.openClaw();
                robot.sleep(200);
                robot.wobbleUp();
                robot.sleep(100);

                drive.followTrajectory(traj[2][1]);
                robot.shootDisc();
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                //SampleMecanumDrive.flywheelFront.set(0);

                drive.followTrajectory(traj[2][2]); //knock

                drive.followTrajectory(traj[2][3]); //backup

                drive.followTrajectory(traj[2][4]); //intake

                drive.followTrajectory(traj[2][5]); //shoot
                robot.shootDisc();
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                robot.shootDisc();
                robot.sleep(300);
                //SampleMecanumDrive.flywheelFront.set(0);

                drive.followTrajectory(traj[2][6]); //intake

                drive.followTrajectory(traj[2][7]); //shoot 1
                robot.shootDisc();
                robot.sleep(300);
                //SampleMecanumDrive.flywheelFront.set(0);
                break;
        }
        telemetry.update();
    }
}