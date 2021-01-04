package org.firstinspires.ftc.teamcode.Team9113;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;

@Config
@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    private double[] milliTime = new double[9];
    private Gamepad[] gamepad = new Gamepad[2];
    private double previousHeading;
    private static double offSetAngle = 0;
    private static double turningSpeed = .75;
    public static int timeThreshold = 350;
    public static int shootThreshold = 80;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
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
            robot.flywheels.run();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double ly = -gamepad1.left_stick_y * robot.drivetrain.currentThrottle;
            double lx = gamepad1.left_stick_x * robot.drivetrain.currentThrottle;
            double rx = -1 * Math.sqrt(Math.abs(gamepad1.right_stick_x)) * gamepad1.right_stick_x * robot.drivetrain.turnThrottle;
            double heading = angles.firstAngle - offSetAngle + 90;
            double speed = Math.hypot(ly, lx);
            double y = speed * Math.sin(Math.atan2(ly, lx) - heading);
            double x = speed * Math.cos(Math.atan2(ly, lx) - heading);
            robot.drivetrain.drivetrain[2].set(-1 * (y-x+rx));
            robot.drivetrain.drivetrain[0].set(-1 * (y+x+rx));
            robot.drivetrain.drivetrain[1].set(-1 * (y-x-rx));
            robot.drivetrain.drivetrain[3].set(-1 * (y+x-rx));
            /*mecanum.driveFieldCentric(x, y, rx, heading + 270, false); */
            if ((gamepad1.right_stick_x <= .01 && gamepad1.left_stick_x <= .01) && (gamepad1.left_stick_y <= 0.01 && gamepad1.left_stick_x <= .01)) {
                robot.drivetrain.brake();
            }
            if (gamepad[0].right_bumper && System.currentTimeMillis() - milliTime[0] > shootThreshold) {
                robot.shootDisc();
                stopwatch(0);
            }
            if (gamepad[0].left_bumper && System.currentTimeMillis() - milliTime[1] > timeThreshold) {
                robot.flywheels.togglePID();
                stopwatch(1);
            }
            if (gamepad1.back) {
                offSetAngle = angles.firstAngle;
            }
            if (gamepad[0].dpad_right && System.currentTimeMillis() - milliTime[2] > 25) {
                robot.drivetrain.mecanumDrive.turn(Math.toRadians(6));
                stopwatch(2);
            }
            if (gamepad[0].dpad_left && System.currentTimeMillis() - milliTime[3] > 25) {
                robot.drivetrain.mecanumDrive.turn(Math.toRadians(-6));
                stopwatch(3);
            }
            if (gamepad[0].dpad_up && System.currentTimeMillis() - milliTime[4] > timeThreshold) {
                robot.flapUpperPosition();
                stopwatch(4);
            }
            if (gamepad[0].dpad_down && System.currentTimeMillis() - milliTime[5] > timeThreshold) {
                robot.flapLowerPosition();
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
            if (gamepad[0].y && System.currentTimeMillis() - milliTime[8] > timeThreshold) {
                robot.toggleWobble();
                stopwatch(8);
            }
            if (gamepad[0].x) {
                robot.reverseIntake();
            } else if (robot.intakeRunning)
                robot.startIntake();
            telemetry.addData("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private void stopwatch(int type) {
        milliTime[type] = System.currentTimeMillis();
    }
}