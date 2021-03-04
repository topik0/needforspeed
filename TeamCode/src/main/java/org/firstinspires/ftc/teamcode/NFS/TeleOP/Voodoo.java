package org.firstinspires.ftc.teamcode.NFS.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NFS.Control.Omnipad;
import org.firstinspires.ftc.teamcode.NFS.GTP.GoToPoint;
import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;
import org.firstinspires.ftc.teamcode.NFS.Vision.AimingVision;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

import static java.lang.Double.NaN;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is the main TeleOP OP Mode, which contains all of the robot controls
 */
@Config
@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    /**
     * The angles for the constant turns
     */
    public static double turnRightAngle = -6, turnLeftAngle = 6;
    /**
     * The timer values for different drivetrain throttles depending on the flywheel and arm states
     */
    public static double flywheelTimerThreshold = 400, armTimerThreshold = 300;
    /**
     * The turn throttle values to be set in certain cases depending on robot component states
     */
    public static double flywheelsTurnThrottle = .5, armTurnThrottle = .6, normalTurnThrottle = .6;
    /**
     * The drivetrain throttle values to be set in certain cases depending on robot component states
     */
    public static double flywheelsDrivetrainThrottle = .9, armDrivetrainThrottle = 1, normalDrivetrainThrottle = 1;
    /**
     * Controls whether or not active breaking is enabled
     */
    //public static double headingP = 1.5, headingD = .1;
    public static double turnAngle = -5;
    public static boolean activeBreakingEnabled = true;
    public static boolean highGoalAngle = true;
    boolean firstLoop = true;
    private static double offSetAngle = 0;
    public ElapsedTime flywheelTimer = new ElapsedTime();
    public ElapsedTime armTimer = new ElapsedTime();
    public static double heading;
    boolean run = false;
    GoToPoint point;
    SampleMecanumDrive drive;
    PIDFController headingPIDF;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        Omnipad pad = new Omnipad(gamepad1, gamepad2, robot);
        drive = new SampleMecanumDrive(hardwareMap);
        point = new GoToPoint(drive, hardwareMap);
        AimingVision aim = new AimingVision(hardwareMap, robot.drivetrain);
     /*
        StopWatch flywheelStopwatch = new StopWatch();
        StopWatch armStopwatch = new StopWatch();
     */
        robot.startPositions();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        BNO055IMU imu = robot.genesis.imu;
        imu.initialize(parameters);
        //headingPIDF = new PIDFController(headingP, 0, headingD, 0);
        waitForStart();
        while (opModeIsActive()) {
            if (firstLoop){
                robot.flap.goToHighGoalPosition();
                highGoalAngle = true;
                firstLoop = false;
            }
            robot.flywheels.run();
            robot.flicker.checkState();
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double ly = pad.getLeftY();
            double lx = pad.getLeftX();
            double rx = pad.getRightX();

            heading = angles.firstAngle - offSetAngle + Math.toRadians(270);
            double speed = Math.hypot(ly, lx);
            double y = pad.getY(speed, heading, ly, lx);
            double x = pad.getX(speed, heading, ly, lx);
            robot.drivetrain.driveFieldCentric(x, y, rx);
            if (activeBreakingEnabled && pad.drivetrainDormant()) robot.drivetrain.brake();
            if (pad.shootRing()) robot.flicker.shootOut();
            if (pad.flywheelsToggle()) {
                robot.flywheels.togglePID();
                flywheelTimer.reset();
            }
            if (pad.setOffset()) offSetAngle = angles.firstAngle;
            if (pad.raiseFlap()) {
                robot.flap.goToHighGoalPosition();
                highGoalAngle = true;
            }

            if (pad.lowerFlap()) {
                robot.flap.goToSlowPowershotPosition();
                highGoalAngle = false;
            }


            //automated powershot

            if (gamepad1.dpad_right) {
                run = true;
                while (run) {
                    point.reset();
                    robot.flap.goToSlowPowershotPosition();
                    robot.flywheels.doVelocity();
                    //first launch
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointPS(0, -25.5, 5, .5, .5);
                        //point.goToPointTele(0, -32, 0);
                        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        break;
                    }
                    robot.delayWithAllPID(200);
                    if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flywheels.setHighGoalVelocity();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        drive.update();
                        break;
                    }
                    robot.flicker.launch();


                    //second launch
                    robot.delayWithAllPID(30);
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointPS(0, -25.5, 0, .5, .5);
                        drive.update();
                        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        break;
                    }
                    robot.delayWithAllPID(100);
                    robot.flicker.launch();
                    robot.delayWithAllPID(30);
                    //third launch
                    point.notDone();
                    while (!point.isDone4 && run) {
                      //  point.goToPointNonBlocking(0, -28, -6);
                        point.goToPointPS(0, -25.5, -7, .5, .5);
                        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        break;
                    }
                    robot.delayWithAllPID(100);
                    robot.flicker.launch();
                    robot.delayWithAllPID(30);
                    robot.flywheels.halt();
                    drive.update();
                    robot.flap.goToHighGoalPosition();
                    highGoalAngle = true;
                    run = false;
                    break;
                }
            }

            if (gamepad1.dpad_left) {
                run = true;
                while (run) {
                    point.reset();
                    robot.flap.goToSlowPowershotPosition();
                    robot.flywheels.doVelocity();
                    //first launch
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointPS(0, -20, 0, .5, .5);
                        //point.goToPointTele(0, -32, 0);
                        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        break;
                    }
                    robot.delayWithAllPID(200);
                    if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flywheels.setHighGoalVelocity();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        drive.update();
                        break;
                    }
                    robot.flicker.launch();


                    //second launch
                    robot.delayWithAllPID(30);
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointPS(0, -25.5, 0, .5, .5);
                        drive.update();
                        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        break;
                    }
                    robot.delayWithAllPID(100);
                    robot.flicker.launch();
                    robot.delayWithAllPID(30);
                    //third launch
                    point.notDone();
                    while (!point.isDone4 && run) {
                        //  point.goToPointNonBlocking(0, -28, -6);
                        point.goToPointPS(0, -32, 0, .5, .5);
                        if (gamepad1.left_trigger > .1 || gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flap.goToHighGoalPosition();
                        highGoalAngle = true;
                        break;
                    }
                    robot.delayWithAllPID(100);
                    robot.flicker.launch();
                    robot.delayWithAllPID(30);
                    robot.flywheels.halt();
                    drive.update();
                    robot.flap.goToHighGoalPosition();
                    highGoalAngle = true;
                    run = false;
                    break;
                }
            }


            /*

            //automated powershot GTP
            if (gamepad1.dpad_right) {
                run = true;
                while (run) {
                    point.reset();
                    robot.flywheels.setPowershotVelocity();
                    robot.flywheels.doPowershotVelocity();
                    robot.flap.goToPowershotPosition();

                    //first launch
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointNonBlocking(0, -20, 0);
                        //point.goToPointTele(0, -32, 0);
                        if (gamepad1.left_trigger > .1) run = false;
                        if (gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flywheels.setHighGoalVelocity();
                        robot.flap.goToHighGoalPosition();
                        drive.update();
                        break;
                    }
                    robot.delayWithAllPID(200);
                    if (gamepad1.left_trigger > .1) run = false;
                    if (gamepad1.right_trigger > .1) run = false;
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flywheels.setHighGoalVelocity();
                        robot.flap.goToHighGoalPosition();
                        drive.update();
                        break;
                    }
                    robot.flicker.launch();


                    //second launch
                    robot.delayWithAllPID(30);
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointNonBlocking(0, -26, 0);
                        if (gamepad1.left_trigger > .1) run = false;
                        if (gamepad1.right_trigger > .1) run = false;
                    }
                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flywheels.setHighGoalVelocity();
                        robot.flap.goToHighGoalPosition();
                        drive.update();
                        break;
                    }
                    robot.delayWithAllPID(100);
                    robot.flicker.launch();
                    robot.delayWithAllPID(30);
                    robot.flap.goToSlowPowershotPosition();
                    robot.delayWithAllPID(20);
                    //third launch
                    point.notDone();
                    while (!point.isDone4 && run) {
                        point.goToPointNonBlocking(0, -32, 0);
                        if (gamepad1.left_trigger > .1) run = false;
                        if (gamepad1.right_trigger > .1) run = false;
                    }

                    if (!run) {
                        robot.flywheels.halt();
                        drive.update();
                        robot.flywheels.setHighGoalVelocity();
                        robot.flap.goToHighGoalPosition();
                        drive.update();
                        break;
                    }
                    robot.delayWithAllPID(100);
                    robot.flicker.launch();
                    robot.delayWithAllPID(30);
                    robot.flywheels.halt();
                    drive.update();
                    robot.flap.goToHighGoalPosition();
                    run = false;
                    break;
                }
            }


             */

                if (gamepad1.right_stick_button) {
                    run = true;
                    while (run) {
                    robot.flap.goToHighGoalPosition();
                    highGoalAngle = true;
                    if (!robot.flywheels.running()) robot.flywheels.togglePID();
                    if (robot.intake.isRunning()) robot.intake.toggle();
                        if(!aim.aiming.seesGoal()) {
                            run = false;
                            break;
                        }
                        point.reset();
                        point.notDone();
                        double goalX = aim.aiming.getGoalCenterX();
                        double turn = Math.atan((160 - (goalX + 10)) / (346.41));
                        while (!point.isDone4 && gamepad1.right_trigger < .1 && gamepad1.left_trigger < .1 && !gamepad1.right_bumper) {
                            point.goToPointNonBlockingTurn(0, 0, Math.toDegrees(turn));
                        }
                        run = false;
                    }
                }

                if (pad.intakeToggle()) robot.intake.toggle();
                if (pad.intakeReverse()) robot.intake.reverse();
                else if (robot.intake.isRunning()) robot.intake.start();
                if (pad.clawToggle()) robot.claw.toggle();
                if (pad.armToggle()) {
                    robot.arm.toggle();
                    if(robot.arm.isUp() && !highGoalAngle){
                    robot.flap.goToPowershotPosition();
                    }
                    else if(robot.arm.isUp() && highGoalAngle){
                        robot.flap.goToHighGoalPosition();
                    }
                    else{
                        robot.flap.goFlush();
                    }
                    armTimer.reset();
                }

                //lowering flap when wobble is out and putting it back up after
            /*

              if (!robot.arm.isUp()) {
                    robot.flap.goFlush();
                } else if (robot.arm.isUp() && highGoalAngle && armTimer.milliseconds() >= 500) {
                    robot.flap.goToHighGoalPosition();
                } else if (robot.arm.isUp() && !highGoalAngle && armTimer.milliseconds() >= 500) {
                    robot.flap.goToPowershotPosition();
                }

                */



                //turn and drivetrain throttles
                if (robot.flywheels.running() && flywheelTimer.milliseconds() >= flywheelTimerThreshold && robot.arm.isUp()) {
                    robot.drivetrain.setTurnThrottle(flywheelsTurnThrottle);
                    robot.drivetrain.setDrivetrainThrottle(flywheelsDrivetrainThrottle);
                    //flywheelStopwatch.reset();
                } else if (!robot.arm.isUp() && armTimer.milliseconds() >= armTimerThreshold) {
                    robot.drivetrain.setTurnThrottle(armTurnThrottle);
                    robot.drivetrain.setDrivetrainThrottle(armDrivetrainThrottle);
                    //armStopwatch.reset();
                } else {
                    robot.drivetrain.setTurnThrottle(normalTurnThrottle);
                    robot.drivetrain.setDrivetrainThrottle(normalDrivetrainThrottle);
                }

                //phone telemetry
                telemetry.addData("x", aim.aiming.getGoalCenterX());
                telemetry.addData("seesGoal", aim.aiming.seesGoal());
//            telemetry.addData("Flicker State", robot.flicker.getState());
//            telemetry.addData("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
//            telemetry.addData("Flywheel Runstate", robot.flywheels.getRunState());
//            telemetry.addData("Flywheel Velostate", robot.flywheels.getVelocityState());
//            telemetry.addData("Arm State", robot.arm.getState());
//            telemetry.addData("Claw State", robot.claw.getState());
//            telemetry.addData("rx", rx);
//            telemetry.addData("turn throttle", robot.drivetrain.getTurnThrottle());
//            telemetry.addData("dt throttle", robot.drivetrain.getThrottle());
               telemetry.update();
            //    telemetry.update();

                //dashboard telemetry
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Flywheel Velocity", robot.flywheels.flywheelFront.getCorrectedVelocity());
                packet.put("Flywheel Target Velocity", robot.flywheels.getTargetVelocity());
                packet.put("Motor Power", robot.flywheels.flywheelFront.motor.getPower());
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
