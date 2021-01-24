package org.firstinspires.ftc.teamcode.NFS.Autonomous;

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

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;
import org.firstinspires.ftc.teamcode.NFS.Vision.Vision;
import org.firstinspires.ftc.teamcode.NFS.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

import java.util.Arrays;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * The normal (High Goal) autonomous
 * Zero Ring: 71 Points
 * One Ring: 83 Points
 * Four Rings: 119 Points
 */
@Config
@Autonomous(name = "NFSAutoHighGoalTurn", group = "Linear Opmode")
public class NFSAutoHighGoalTurn extends LinearOpMode {
    private Trajectory build;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, true);
        Vision vision = new Vision(hardwareMap, telemetry);
        robot.startPositions();
        SampleMecanumDrive drive = robot.drivetrain.mecanumDrive;
        Pose2d startPose = new Pose2d(-63, -44, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        Trajectory[][] traj = new Trajectory[3][12];
        traj[0][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-2, -57), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.31))
                .addTemporalMarker(.75, () -> robot.intake.down())
                .addTemporalMarker(.75, () -> robot.arm.down())
                .addTemporalMarker(1.85, () -> robot.claw.open())
                .addTemporalMarker(1.9, () -> robot.arm.up())
                .build();

        traj[0][1] = drive.trajectoryBuilder(traj[0][0].end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .splineTo(new Vector2d(-6, -18), Math.toRadians(-13))
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
              //  .addDisplacementMarker(() -> robot.flicker.launch())
                .build();
        traj[0][2] = drive.trajectoryBuilder(traj[0][1].end(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(1.5, () -> robot.arm.down())
                .splineTo(new Vector2d(-32, -24), Math.toRadians(-132))
                .build();
        traj[0][3] = drive.trajectoryBuilder(traj[0][2].end(), false)
                .splineTo(new Vector2d(-2, -48), Math.toRadians(-20))
                .addTemporalMarker(.9, () -> robot.arm.down())
                .build();
        traj[0][4] = drive.trajectoryBuilder(traj[0][3].end(), true) //not used
                .splineTo(new Vector2d(-12, -36), Math.toRadians(-45))
                .build();
        traj[0][5] = drive.trajectoryBuilder(traj[0][3].end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .splineTo(new Vector2d(12, -24), Math.toRadians(0))
                .build();
        traj[1][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.29))
                .addTemporalMarker(.75, () -> robot.intake.down())
               // .splineTo(new Vector2d(17.5, -42), Math.toRadians(30))
                .splineTo(new Vector2d(21, -44), Math.toRadians(30))
                .addTemporalMarker(1.5, () -> robot.arm.down())
                .addDisplacementMarker(() -> drive.flywheels.doMaxVelocity())
                .build();
        traj[1][1] = drive.trajectoryBuilder(traj[1][0].end(), true)
                .splineTo(new Vector2d(-6, -18), Math.toRadians(160))
                .build();
        traj[1][2] = drive.trajectoryBuilder(traj[1][1].end(), false)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .addTemporalMarker(.1, () -> {
                    robot.arm.down();
                    robot.flap.setPosition(0.29);
                    robot.claw.close();
                })
                .addTemporalMarker(1.4, () -> robot.claw.open())
                .splineTo(new Vector2d(-32, -24), Math.toRadians(-136)) //might need to change heading
                .build();
        traj[1][3] = drive.trajectoryBuilder(traj[1][2].end(), false)
                .splineTo(new Vector2d(-24, -55), Math.toRadians(-20))
                .splineTo(new Vector2d(13, -42), Math.toRadians(45))
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
                })
                .build();
        traj[1][6] = drive.trajectoryBuilder(traj[1][5].end(), false)
                .splineTo(new Vector2d(12, -36), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> drive.flywheels.halt())
                .build();


        traj[2][0] = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))
                .splineTo(new Vector2d(40, -55), Math.toRadians(0))
                .addTemporalMarker(0.1, () -> robot.flap.setPosition(.29))
                .addTemporalMarker(.75, () -> robot.intake.down())
                .addTemporalMarker(2, () -> robot.arm.down())
                .build();
        traj[2][1] = drive.trajectoryBuilder(traj[2][0].end(), true)
                .splineTo(new Vector2d(-6, -18), Math.toRadians(158))
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
                .build();
        traj[2][2] = drive.trajectoryBuilder(traj[2][1].end(), true)
                .splineTo(new Vector2d(-24, -18), Math.toRadians(270))
                .addDisplacementMarker(() -> robot.intake.start())
                .build();
        traj[2][3] = drive.trajectoryBuilder(traj[2][2].end(), true)
                .splineTo(new Vector2d(-24, -32), Math.toRadians(270),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-24, -44), Math.toRadians(270),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        traj[2][4] = drive.trajectoryBuilder(traj[2][3].end().plus(new Pose2d(0, 0, Math.toRadians(70))))
                .splineTo(new Vector2d(-37, -40), Math.toRadians(165))
                .addDisplacementMarker(() -> {
                    robot.claw.close();
                    robot.intake.stop();
                })
                .build();
        traj[2][5] = drive.trajectoryBuilder(traj[2][4].end().plus(new Pose2d(0, 0, Math.toRadians(-165))), false)
                .splineTo(new Vector2d(-4.5, -40), Math.toRadians(-1.5))
                .addTemporalMarker(.1, () -> {
                    drive.flywheels.doMaxVelocity();
                    robot.flap.setPosition(.29);
                })
                .build();
        traj[2][6] = drive.trajectoryBuilder(traj[2][5].end(), true)
                .splineTo(new Vector2d(-24, -55), Math.toRadians(-139))
                .addTemporalMarker(.1, () -> robot.intake.start())
                .build();
        traj[2][7] = drive.trajectoryBuilder(traj[2][6].end(), false)
                .splineTo(new Vector2d(-4.5, -40), Math.toRadians(-0.5))
                .addDisplacementMarker(() -> robot.intake.stop())
                .addTemporalMarker(.1, () -> drive.flywheels.doMaxVelocity())
                .build();
        traj[2][8] = drive.trajectoryBuilder(traj[2][7].end(), false)
                .splineTo(new Vector2d(44, -46), Math.toRadians(-20))
                .addTemporalMarker(.75, () -> robot.arm.down())
                .addDisplacementMarker(() -> robot.claw.open())
                .build();
        traj[2][9] = drive.trajectoryBuilder(traj[2][8].end(), true)
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
                .addTemporalMarker(.1, () -> {
                    robot.arm.up();
                    robot.claw.close();
                })
                .build();

        while (!isStarted() && vision.isNotReadyToRead()) {
            telemetry.addData("Vision Status", "Not Ready");
            telemetry.update();
        }
        UGContourRingPipeline.Height height = vision.getHeight();
        while (!isStarted() && vision.isNotReady()) {
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

              //wobble drop
                drive.followTrajectory(traj[0][0]);
                drive.turn(Math.toRadians(90));
                robot.delayWithAllPID(100);

               //shoot
                drive.followTrajectory(traj[0][1]);
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                drive.flywheels.halt();

                //wobble grab
                drive.followTrajectory(traj[0][2]);
                robot.claw.close();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(750);

                //wobble drop
                drive.followTrajectory(traj[0][3]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(750);
                robot.claw.close();

                //park
                drive.turn(Math.toRadians(90));
                robot.delayWithAllPID(100);
                drive.followTrajectory(traj[0][5]);
                break;
            case ONE:
                telemetry.addData("There is one ring", "");

               //1st wobble drop
                drive.followTrajectory(traj[1][0]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(200);

              //shoot 3
                drive.followTrajectory(traj[1][1]);
                robot.delayWithAllPID(200);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(200);
                drive.flywheels.halt();

                //wobble grab
                drive.followTrajectory(traj[1][2]);
                robot.claw.close();
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(750);

                //wobble drop
                drive.followTrajectory(traj[1][3]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(750);
                robot.claw.close();


                drive.followTrajectory(traj[1][4]);

                //shoot
                drive.followTrajectory(traj[1][5]);
                robot.intake.reverse();
                robot.delayWithAllPID(50);
                robot.intake.start();
                robot.delayWithAllPID(300);
                robot.intake.stop();
                robot.delayWithAllPID(100);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();

                //park
                drive.followTrajectory(traj[1][6]);
                break;


            case FOUR:
                telemetry.addData("There are four rings", "");
                //first wobble drop
                drive.followTrajectory(traj[2][0]);
                robot.claw.open();
                robot.delayWithAllPID(200);
                robot.arm.up();
                robot.delayWithAllPID(100);

                //shoot highgoal (3 rings)
                drive.followTrajectory(traj[2][1]);
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(100);
                drive.flywheels.halt();

                //intake stack (3 rings)
                drive.followTrajectory(traj[2][2]);
                drive.followTrajectory(traj[2][3]);

                //wobble grab
                robot.arm.down();
                drive.turn(Math.toRadians(70));
                drive.followTrajectory(traj[2][4]);
                robot.delayWithAllPID(300);
                robot.arm.up();
                robot.delayWithAllPID(300);
                drive.turn(Math.toRadians(-165));

                //shoot high (3 rings)
                drive.followTrajectory(traj[2][5]);
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                robot.delayWithAllPID(400);
                robot.flicker.launch();
                robot.delayWithAllPID(300);
                robot.flicker.launch();
                robot.delayWithAllPID(100);
                drive.flywheels.halt();

                //intake stack (1 ring)
                drive.followTrajectory(traj[2][6]);

                //shoot high (1 ring)
                drive.followTrajectory(traj[2][7]);
                robot.intake.reverse();
                robot.delayWithAllPID(100);
                robot.intake.start();
                robot.delayWithAllPID(150);
                robot.intake.stop();
                robot.delayWithAllPID(50);
                robot.flicker.launch();
                robot.delayWithAllPID(150);
                robot.flicker.launch();
                robot.delayWithAllPID(100);
                drive.flywheels.halt();

                //second wobble drop
                drive.followTrajectory(traj[2][8]);
                robot.delayWithAllPID(200);

                //park
                drive.followTrajectory(traj[2][9]);
                break;
        }
        telemetry.update();
    }
}