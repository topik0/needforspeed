package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arcadia", group = "Linear Opmode")
public class Arcadia extends LinearOpMode {
    double milliTime, milliTimeTwo;

    @Override
    public void runOpMode() {
        // Intialize variables
        boolean capstoneArmUp = true;
        boolean intakeStopped = true;
        boolean jointUp = true;
        double leftPower, rightPower, xValue, yValue, rightYVal;
        boolean goingIn;
        double speed = 1.0;
        boolean clawUp = true;
        // Initialize components
        DcMotor motorZero = hardwareMap.dcMotor.get("motorZero");
        DcMotor motorOne = hardwareMap.dcMotor.get("motorOne");
        DcMotor motorTwo = hardwareMap.dcMotor.get("motorTwo");
        DcMotor motorThree = hardwareMap.dcMotor.get("motorThree");
        DcMotor motorFour = hardwareMap.dcMotor.get("motorFour");
        DcMotor motorFive = hardwareMap.dcMotor.get("motorFive");
        DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");
        Servo capstoneServo = hardwareMap.servo.get("capstone");
        Servo jointServo = hardwareMap.servo.get("servoOne");
        Servo shoulderServo = hardwareMap.servo.get("servoTwo");
        Servo platformServoOne = hardwareMap.servo.get("foundationZero");
        Servo platformServoTwo = hardwareMap.servo.get("foundationOne");

        // Set things to starting positions
        platformServoOne.setPosition(.3);
        platformServoTwo.setPosition(.7);
        jointServo.setPosition(.25);
        capstoneServo.setPosition(1);
        shoulderServo.setPosition(.02);
        waitForStart();
        // Reset encoder count kept by left motor.
        // slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // telemetry.addData("encoder-fwd", slideMotor.getCurrentPosition() + "  busy=" + slideMotor.isBusy());
        //  telemetry.update();
        if (opModeIsActive())
            while (opModeIsActive()) {
                yValue = gamepad1.left_stick_y * -1;
                xValue = gamepad1.left_stick_x * -1;
                rightYVal = gamepad1.right_stick_y;
                leftPower = yValue - xValue;
                rightPower = yValue + xValue;
                goingIn = true;
                if (rightYVal != 0 && (yValue == 0 && xValue == 0)) {
                    if (xValue > 0)
                        leftPower *= xValue = 1;
                    if (xValue < 0)
                        rightPower *= xValue = 1;
                }
                /**if(xValue > 0) {
                 motorZero.setPower(rightYVal);
                 motorZero.setPower(-rightYVal);
                 motorZero.setPower(rightYVal);
                 motorZero.setPower(-rightYVal);
                 }
                 if(xValue < 0) {
                 motorZero.setPower(-rightYVal);
                 motorZero.setPower(rightYVal);
                 motorZero.setPower(-rightYVal);
                 motorZero.setPower(rightYVal);
                 }**/
                motorZero.setPower(Range.clip((leftPower * rightYVal), -1.0, 1.0));
                motorOne.setPower(-Range.clip((rightPower * rightYVal), -1.0, 1.0));
                motorTwo.setPower(Range.clip((leftPower * rightYVal), -1.0, 1.0));
                motorThree.setPower(-Range.clip((rightPower * rightYVal), -1.0, 1.0));
                if (gamepad1.dpad_left) {
                    motorZero.setPower(-.4);
                    motorOne.setPower(-.4);
                    motorTwo.setPower(.4);
                    motorThree.setPower(.4);
                }
                if (gamepad1.dpad_right) {
                    motorZero.setPower(.4);
                    motorOne.setPower(.4);
                    motorTwo.setPower(-.4);
                    motorThree.setPower(-.4);
                }
                if (gamepad1.dpad_up) {
                    motorZero.setPower(-.4);
                    motorOne.setPower(.4);
                    motorTwo.setPower(-.4);
                    motorThree.setPower(.4);
                }
                if (gamepad1.dpad_down) {
                    motorZero.setPower(.4);
                    motorOne.setPower(-.4);
                    motorTwo.setPower(.4);
                    motorThree.setPower(-.4);
                }
                if (gamepad1.a) {
                    motorZero.setPower(.4);
                    motorThree.setPower(-.4);
                }
                if (gamepad1.y) {
                    motorZero.setPower(-.4);
                    motorThree.setPower(.4);
                }
                if (gamepad1.x) {
                    motorOne.setPower(-.4);
                    motorTwo.setPower(.4);
                }
                if (gamepad1.b) {
                    motorOne.setPower(.4);
                    motorTwo.setPower(-.4);
                }
                if (gamepad1.left_trigger > 0) {
                    motorZero.setPower(.7);
                    motorThree.setPower(-.7);
                }
                if (gamepad1.right_trigger > 0) {
                    motorZero.setPower(-.7);
                    motorThree.setPower(.7);
                }
                // Inverts the goingIn boolean that changes the direction of the intake motors
                if (gamepad1.back)
                    goingIn = !goingIn;
                // Stops the intake if the right bumper is pressed.
                if (gamepad1.right_bumper && System.currentTimeMillis() - milliTime > 250) {
                    intakeStopped = !intakeStopped;
                    stopwatch(1);
                }
                if (gamepad2.dpad_up && slideMotor.getCurrentPosition() < 7730)
                    slideMotor.setPower(-.7);
                    // Allows the slide motor to move down if shoulderServo is out.  Sets different val
                else if (gamepad2.dpad_down && slideMotor.getCurrentPosition() > 150)
                    slideMotor.setPower(.7);
                else
                    slideMotor.setPower(0);
                // Makes the shoulder servo move in (not facing intake)
                if (gamepad2.dpad_left && slideMotor.getCurrentPosition() >= 2300)
                    shoulderServo.setPosition(.02);
                // Makes the shoulder servo move out (facing intake)
                if (gamepad2.dpad_right && slideMotor.getCurrentPosition() >= 2300)
                    shoulderServo.setPosition(.685);
                // Makes the joint (clamp) servo move up
                if (gamepad2.y && System.currentTimeMillis() - milliTimeTwo > 250) {
                    if (clawUp)
                        jointServo.setPosition(.5);
                    if (!clawUp)
                        jointServo.setPosition(.2);
                    clawUp = !clawUp;
                    stopwatch(2);
                }
                // Makes the intake go in if the intake motors are currently going outwards
                if (!intakeStopped && !goingIn) {
                    motorFour.setPower(-.4);
                    motorFive.setPower(.4);
                }
                // Makes the intake go out (reverse) if the intake motors are currently going inwards
                if (!intakeStopped && goingIn) {
                    motorFour.setPower(.4);
                    motorFive.setPower(-.4);
                }
                // Makes the intake stopped if intakeStopped is true
                if (intakeStopped) {
                    motorFour.setPower(0);
                    motorFive.setPower(0);
                }
                if (gamepad2.right_bumper) {
                    platformServoOne.setPosition(.3);
                    platformServoTwo.setPosition(.7);
                }
                if (gamepad2.left_bumper) {
                    platformServoOne.setPosition(0);
                    platformServoTwo.setPosition(1);
                }
                // Makes the capstone servo go down
                if (gamepad2.back)
                    capstoneServo.setPosition(-.6);
                if (gamepad2.start)
                    capstoneServo.setPosition(1);
                if (gamepad1.start) {
                    if (speed == 1.0)
                        speed = .4;
                    if (speed == .4)
                        speed = 1.0;
                }
                if (gamepad2.left_stick_y >= .3)
                    capstoneServo.setPosition(1);
                else if (gamepad2.right_stick_y >= .3)
                    capstoneServo.setPosition(.3);
                telemetry.addData("Pos", "Slide: " + slideMotor.getCurrentPosition());
                telemetry.addData("Pos", "Servos " + platformServoOne.getPosition());
                telemetry.addData("Pos", "Servos2 " + platformServoTwo.getPosition());
                telemetry.update();
            }
    }

    private void stopwatch(int type) {
        if (type == 1)
            milliTime = System.currentTimeMillis();
        if (type == 2)
            milliTimeTwo = System.currentTimeMillis();
    }
}
