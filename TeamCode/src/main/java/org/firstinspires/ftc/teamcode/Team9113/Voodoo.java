package org.firstinspires.ftc.teamcode.Team9113;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    private double[] milliTime = new double[9];
    private Gamepad[] gamepad = new Gamepad[2];
    private double previousHeading;
    private double offSetAngle = 0;
    private double turningSpeed = .75;
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        gamepad[0] = gamepad1;
        gamepad[1] = gamepad2;
        // Set things to starting positions
        robot.startPositions();
        // Initialize variables
        final int timeThreshold = 350;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        MecanumDrive mecanum = new MecanumDrive(robot.drivetrain.drivetrain[0], robot.drivetrain.drivetrain[1], robot.drivetrain.drivetrain[2], robot.drivetrain.drivetrain[3]);
        waitForStart();
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double ly = -gamepad1.left_stick_y * robot.drivetrain.currentThrottle;
            double lx = gamepad1.left_stick_x * robot.drivetrain.currentThrottle;
            double rx = -gamepad1.right_stick_x * robot.drivetrain.turnThrottle;
            double heading = angles.firstAngle - offSetAngle;
            double speed = Math.hypot(ly, lx);
            double y = speed * Math.sin(Math.atan2(ly, lx) - heading);
            double x = speed * Math.cos(Math.atan2(ly, lx) - heading);
            mecanum.driveFieldCentric(x, y, rx, heading + 270, false);
            if (gamepad[0].right_bumper && System.currentTimeMillis() - milliTime[0] > 85) {
                robot.shootDisc();
                stopwatch(0);
            }
            if (gamepad[0].left_bumper && System.currentTimeMillis() - milliTime[1] > timeThreshold) {
                robot.toggleFlywheels();
                robot.drivetrain.toggleNormalDrive();
                if (turningSpeed >= .75)
                    turningSpeed = .5;
                else
                    turningSpeed = .75;
                stopwatch(1);
            }
            if (gamepad1.back) {
                offSetAngle = angles.firstAngle;
            }
            if (gamepad[0].dpad_right && System.currentTimeMillis() - milliTime[2] > 25) {
                robot.flapAdjustUp();
                robot.setFlywheelsModeNormal();
                stopwatch(2);
            }
            if (gamepad[0].dpad_left && System.currentTimeMillis() - milliTime[3] > 25) {
                robot.flapAdjustDown();
                robot.setFlywheelsModeSlow();
                stopwatch(3);
            }
            if (gamepad[0].dpad_up && System.currentTimeMillis() - milliTime[4] > timeThreshold) {
                robot.flapUpperPosition();
                robot.flywheelsSlow = false;
                stopwatch(4);
            }
            if (gamepad[0].dpad_down && System.currentTimeMillis() - milliTime[5] > timeThreshold) {
                robot.flapLowerPosition();
                robot.flywheelsSlow = true;
                stopwatch(5);
            }
            if (gamepad[0].a && System.currentTimeMillis() - milliTime[6] > 500) {
                robot.toggleIntake();
                stopwatch(6);
            }
            if (gamepad[0].b && System.currentTimeMillis() - milliTime[7] > timeThreshold) {
                robot.toggleClaw();
                stopwatch(7);
            }
            if (gamepad[0].start) {
                robot.flap.setPosition(.3);
                sleep(100);
            }
            if (gamepad[0].y && System.currentTimeMillis() - milliTime[8] > 50) {
                robot.toggleWobble();
                stopwatch(8);
            }
            if (gamepad[0].x) {
                robot.reverseIntake();
            } else if (robot.intakeRunning)
                robot.startIntake();
            telemetry.update();
        }
    }

    private void stopwatch(int type) {
        milliTime[type] = System.currentTimeMillis();
    }
}