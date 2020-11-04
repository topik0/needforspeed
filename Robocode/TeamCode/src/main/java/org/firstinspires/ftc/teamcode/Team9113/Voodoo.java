package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    private double[] milliTime = new double[9];
    private Gamepad[] gamepad = new Gamepad[2];

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
            if (gamepad[0].right_bumper && System.currentTimeMillis() - milliTime[0] > timeThreshold) {
                robot.shootDisc();
                stopwatch(0);
            }
            if (gamepad[0].left_bumper && System.currentTimeMillis() - milliTime[1] > timeThreshold) {
                robot.setFlywheelPower(1);
                stopwatch(1);
            }
            if (gamepad[0].x && System.currentTimeMillis() - milliTime[2] > timeThreshold) {
                robot.setFlywheelPower(0);
                stopwatch(2);
            }
            telemetry.update();
        }
    }

    private void stopwatch(int type) {
        milliTime[type] = System.currentTimeMillis();
    }
}
