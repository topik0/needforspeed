package org.firstinspires.ftc.teamcode.Team9113;

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
    private double ly, lx, rx, x, y, heading, speed, previousHeading;
    private double offSetAngle = 0;
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        gamepad[0] = gamepad1;
        gamepad[1] = gamepad2;
        // Set things to starting positions
        if (opModeIsActive()) robot.startPositions();
        // Initialize variables
        final int timeThreshold = 350;
        waitForStart();
        while (opModeIsActive()) {
            ly = -gamepad1.left_stick_y;
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            //imu
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            heading = angles.firstAngle - offSetAngle  /* + previousHeading */;
            speed = Math.hypot(ly, lx);
            if (gamepad1.start) {
                offSetAngle = angles.firstAngle /* + previousHeading */;
            }

            // imu translations
            y = speed * Math.sin(Math.atan2(ly, lx) - heading);
            x = speed * Math.cos(Math.atan2(ly, lx) - heading);

            // Send calculated power to wheels
            robot.drivetrain.setFieldCentricPower(y, x, rx);
            if (gamepad[0].right_bumper && System.currentTimeMillis() - milliTime[0] > 100) {
                robot.shootDisc();
                stopwatch(0);
            }
            if (gamepad[0].left_bumper && System.currentTimeMillis() - milliTime[1] > timeThreshold) {
                robot.startFlywheels();
                stopwatch(1);
            }
            if (gamepad[0].x && System.currentTimeMillis() - milliTime[2] > timeThreshold) {
                robot.stopFlywheels();
                stopwatch(2);
            }
            telemetry.update();
        }
    }

    private void stopwatch(int type) {
        milliTime[type] = System.currentTimeMillis();
    }
}
