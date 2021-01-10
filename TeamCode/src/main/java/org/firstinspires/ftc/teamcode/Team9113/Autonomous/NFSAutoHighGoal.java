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

import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Vision;
import org.firstinspires.ftc.teamcode.Team9113.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Team9113.drive.SampleMecanumDrive;

import java.util.Arrays;

@Config
@Autonomous(name = "NFSAutoHighGoal", group = "Linear Opmode")
public class NFSAutoHighGoal extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Vision vision = new Vision(hardwareMap, telemetry);
        robot.startPositions();
        SampleMecanumDrive drive = robot.drivetrain.mecanumDrive;
        Pose2d startPose = new Pose2d(-63, -44, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][10];
        traj[0][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-5, -57), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.32))
                .addTemporalMarker(.25, () -> robot.intake.down())
                .addTemporalMarker(.75, () -> robot.arm.down())
                .addTemporalMarker(1.85, () -> robot.claw.open())
                .addTemporalMarker(1.9, () -> robot.arm.up())
                .build();
        traj[0][1] = drive.trajectoryBuilder(traj[0][0].end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(-13))
                .addTemporalMarker(.2, () -> drive.flywheels.doMaxVelocity())
                .build();
        traj[0][2] = drive.trajectoryBuilder(traj[0][1].end(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(1.5, () -> robot.arm.up())
                .splineTo(new Vector2d(-32, -24), Math.toRadians(-135))
                .build();
        traj[0][3] = drive.trajectoryBuilder(traj[0][2].end(), false)
                .splineTo(new Vector2d(-2, -48), Math.toRadians(-20))
                .addTemporalMarker(.7, () -> robot.arm.down())
                .build();
        traj[0][4] = drive.trajectoryBuilder(traj[0][3].end(), true)
                .splineTo(new Vector2d(-12, -36), Math.toRadians(-45))
                .build();
        traj[0][5] = drive.trajectoryBuilder(traj[0][3].end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .splineTo(new Vector2d(12, -24), Math.toRadians(0))
                .build();
        traj[1][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.32))
                .addTemporalMarker(.25, () -> robot.intake.down())
                .splineTo(new Vector2d(17.5, -42), Math.toRadians(30))
                .addTemporalMarker(1.5, () -> robot.arm.down())
                .addDisplacementMarker(() -> drive.flywheels.doMaxVelocity())
                .build();
        traj[1][1] = drive.trajectoryBuilder(traj[1][0].end(), true)
                .splineTo(new Vector2d(-6, -12), Math.toRadians(164))
                .build();
        traj[1][2] = drive.trajectoryBuilder(traj[1][1].end(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(.1, () -> {
                    robot.arm.down();
                    robot.flap.setPosition(0.28);
                    robot.claw.open();
                })
                .splineTo(new Vector2d(-32, -24), Math.toRadians(-138))
                .build();
        traj[1][3] = drive.trajectoryBuilder(traj[1][2].end(), false)
                .splineTo(new Vector2d(-24, -55), Math.toRadians(-20))
                .splineTo(new Vector2d(16, -32), Math.toRadians(45))
                .addTemporalMarker(1.75, () -> robot.arm.down())
                .addDisplacementMarker(() -> robot.intake.start())
                .build();
        traj[1][4] = drive.trajectoryBuilder(traj[1][3].end(), true)
                .splineTo(new Vector2d(-24, -36), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.flywheels.doMaxVelocity())
                .build();
        traj[1][5] = drive.trajectoryBuilder(traj[1][4].end(), false)
                .splineTo(new Vector2d(-2, -36), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.intake.stop();
                    robot.flicker.launch();
                })
                .build();
        traj[1][6] = drive.trajectoryBuilder(traj[1][4].end(), false)
                .splineTo(new Vector2d(12, -36), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> drive.flywheels.halt())
                .build();
        traj[2][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .splineTo(new Vector2d(40, -55), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.29))
                .addTemporalMarker(.25, () -> robot.intake.down())
                .addTemporalMarker(2, () -> robot.arm.down())
                .build();
        traj[2][1] = drive.trajectoryBuilder(traj[2][0].end(), true)
                .splineTo(new Vector2d(-4.5, -36), Math.toRadians(170))
                .addTemporalMarker(.5, () -> drive.flywheels.doMaxVelocity())
                .build();
        traj[2][2] = drive.trajectoryBuilder(traj[2][1].end(), true)
                .addTemporalMarker(.1, () -> robot.intake.start())
                .splineTo(new Vector2d(-20, -36), Math.toRadians(180),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-32, -36), Math.toRadians(180),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        traj[2][3] = drive.trajectoryBuilder(traj[2][2].end(), false)
                .splineTo(new Vector2d(-4.5, -36), Math.toRadians(-6))
                .addDisplacementMarker(() -> robot.intake.stop())
                .addTemporalMarker(.5, () -> drive.flywheels.doMaxVelocity())
                .build();
        traj[2][4] = drive.trajectoryBuilder(traj[2][3].end(), true)
                .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                .addTemporalMarker(.1, () -> robot.intake.start())
                .build();
        traj[2][5] = drive.trajectoryBuilder(traj[2][4].end().plus(new Pose2d(0, 0, Math.toRadians(-165))), false)
                .splineTo(new Vector2d(-4.5, -40), Math.toRadians(-2.5))
                .addTemporalMarker(.1, () -> {
                    drive.flywheels.doMaxVelocity();
                    robot.flap.setPosition(.28);
                })
                .build();
        traj[2][6] = drive.trajectoryBuilder(traj[2][5].end(), true)
                .splineTo(new Vector2d(-24, -55), Math.toRadians(225))
                .addTemporalMarker(.1, () -> robot.intake.start())
                .build();
        traj[2][7] = drive.trajectoryBuilder(traj[2][6].end(), false)
                .splineTo(new Vector2d(-4.5, -40), Math.toRadians(-2.5))
                .addDisplacementMarker(() -> robot.intake.stop())
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
                .build();
        traj[2][8] = drive.trajectoryBuilder(traj[2][7].end(), false)
                .splineTo(new Vector2d(44, -44), Math.toRadians(-15))
                .addTemporalMarker(.75, () -> robot.arm.down())
                .build();
        traj[2][9] = drive.trajectoryBuilder(traj[2][8].end(), true)
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
                .addTemporalMarker(.1, () -> {
                    robot.arm.up();
                    robot.claw.close();
                })
                .build();
        while (!isStarted() && !vision.isReadyToRead())
            telemetry.addData("Vision Status", "Not Ready");
        UGContourRingPipeline.Height height = vision.getHeight();
        while (!isStarted() && vision.isReady()) {
            height = vision.getHeight();
            telemetry.addData("Vision Status", "Not Ready");
            telemetry.addData("Camera Initialization Time: ", vision.cameraInitTime());
            telemetry.update();
        }
        while (!isStarted()) {
            height = vision.getHeight();
            telemetry.addData("Vision Status", "Ready");
            telemetry.addData("Ring count: ", height);
            telemetry.addData("Camera Initialization Time: ", vision.cameraInitTime());
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPose);
        switch (height) {
            case ZERO:
                telemetry.addData("There are no rings", "");
                drive.followTrajectory(traj[0][0]);
                drive.turn(Math.toRadians(90));
                robot.delayWithAllPID(100);
                drive.followTrajectory(traj[0][1]);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                drive.turn(Math.toRadians(7.5));
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                drive.turn(Math.toRadians(7.5));
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                drive.flywheels.halt();
                drive.followTrajectory(traj[0][2]);
                robot.claw.close();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(750);
                drive.followTrajectory(traj[0][3]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(750);
                robot.claw.close();
                drive.turn(Math.toRadians(90));
                robot.delayWithAllPID(100);
                drive.followTrajectory(traj[0][5]);
                break;
            case ONE:
                telemetry.addData("There is one ring", "");
                drive.followTrajectory(traj[1][0]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(200);
                drive.followTrajectory(traj[1][1]);
                robot.flicker.launch();
                drive.turn(Math.toRadians(7));
                robot.delayWithAllPID(500);
                robot.flicker.launch();
                drive.turn(Math.toRadians(7));
                robot.delayWithAllPID(500);
                robot.flicker.launch();
                robot.delayWithAllPID(200);
                drive.flywheels.halt();
                drive.followTrajectory(traj[1][2]);
                robot.claw.close();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(750);
                drive.followTrajectory(traj[1][3]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(750);
                robot.claw.open();
                drive.followTrajectory(traj[1][4]);
                drive.followTrajectory(traj[1][5]);
                drive.followTrajectory(traj[1][6]);
                break;
            case FOUR:
                telemetry.addData("There are four rings", "");
                drive.followTrajectory(traj[2][0]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(100);
                drive.followTrajectory(traj[2][1]);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                drive.flywheels.halt();
                drive.followTrajectory(traj[2][2]);
                drive.followTrajectory(traj[2][3]);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                drive.flywheels.halt();
                drive.followTrajectory(traj[2][4]);
                break;
        }
        telemetry.update();
    }
}