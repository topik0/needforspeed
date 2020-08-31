package org.firstinspires.ftc.teamcode.Team9113;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LoneSoldier", group = "Linear Opmode")
public class LoneSoldier extends LinearOpMode {
    private double[] milliTime = new double[6];
    @Override
    public void runOpMode() {
        // Initialize variables
        boolean capstoneArmUp = true;
        boolean intakeStopped = true;
        boolean jointUp = true;
        boolean clawUp = true;
        boolean hyperDrive = false;
        boolean goingIn;
        boolean platformDown = false;
        boolean capstonePlaced = false;
        boolean shoulderServoOut = false;
        double leftPower, rightPower;
        double speed = .9;
        final int timeThreshold = 250;
        final double intakeSpeed = .35;
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
        capstoneServo.setPosition(.5);
        shoulderServo.setPosition(.02);
        waitForStart();
        //  telemetry.update();
        if (opModeIsActive())
            while (opModeIsActive()) {
                rightPower = gamepad1.right_stick_y*speed;
                leftPower = gamepad1.left_stick_y*speed;
                goingIn = true;
                motorZero.setPower(leftPower);
                motorOne.setPower(-rightPower);
                motorTwo.setPower(leftPower);
                motorThree.setPower(-rightPower);
                if (gamepad1.dpad_left) {
                    motorZero.setPower(-speed);
                    motorOne.setPower(-speed);
                    motorTwo.setPower(speed);
                    motorThree.setPower(speed);
                } if (gamepad1.dpad_right) {
                    motorZero.setPower(speed);
                    motorOne.setPower(speed);
                    motorTwo.setPower(-speed);
                    motorThree.setPower(-speed);
                } if (gamepad1.dpad_up) {
                    motorZero.setPower(-speed);
                    motorOne.setPower(speed);
                    motorTwo.setPower(-speed);
                    motorThree.setPower(speed);
                } if (gamepad1.dpad_down) {
                    motorZero.setPower(speed);
                    motorOne.setPower(-speed);
                    motorTwo.setPower(speed);
                    motorThree.setPower(-speed);
                }
                // Inverts the goingIn boolean that changes the direction of the intake motors
                if(gamepad1.back)
                    goingIn = !goingIn;
                // Stops the intake if the right bumper is pressed.
                if(gamepad1.right_bumper && System.currentTimeMillis()-milliTime[0] > timeThreshold) {
                    intakeStopped = !intakeStopped;
                    stopwatch(1);
                }
                if(gamepad1.y /*&& slideMotor.getCurrentPosition() < 7280*/)
                    slideMotor.setPower(-1);
                    // Allows the slide motor to move down if shoulderServo is out.  Sets different val
                else if(gamepad1.x/* && slideMotor.getCurrentPosition() > -150*/)
                    slideMotor.setPower(1);
                else {
                    slideMotor.setPower(0);
                    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                // Makes the shoulder servo move in (not facing intake)
                if(gamepad1.b&& (shoulderServoOut && System.currentTimeMillis()-milliTime[5] > timeThreshold)) {
                    shoulderServoOut = false;
                    shoulderServo.setPosition(.02);
                    stopwatch(6);
                }
                if(gamepad1.b&& (!shoulderServoOut && System.currentTimeMillis()-milliTime[5] > timeThreshold)) {
                    shoulderServoOut = true;
                    shoulderServo.setPosition(.685);
                    stopwatch(6);
                }
                // Makes the joint (clamp) servo move up
                if(gamepad1.a && System.currentTimeMillis()-milliTime[1] > timeThreshold){
                    if(clawUp)
                        jointServo.setPosition(.5);
                    if(!clawUp)
                        jointServo.setPosition(.2);
                    clawUp = !clawUp;
                    stopwatch(2);
                }
                // Makes the intake go in if the intake motors are currently going outwards
                if(!intakeStopped && !goingIn){
                    motorFour.setPower(-intakeSpeed);
                    motorFive.setPower(intakeSpeed);
                }
                // Makes the intake go out (reverse) if the intake motors are currently going inwards
                if(!intakeStopped && goingIn){
                    motorFour.setPower(intakeSpeed);
                    motorFive.setPower(-intakeSpeed);
                }
                // Makes the intake stopped if intakeStopped is true
                if(intakeStopped){
                    motorFour.setPower(0);
                    motorFive.setPower(0);
                }
                if(gamepad1.start && (platformDown && System.currentTimeMillis()-milliTime[3] > timeThreshold)) {
                    platformDown = false;
                    platformServoOne.setPosition(.3);
                    platformServoTwo.setPosition(.7);
                    stopwatch(4);
                }
                if(gamepad1.start && (!platformDown && System.currentTimeMillis()-milliTime[3] > timeThreshold)) {
                    platformDown = true;
                    platformServoOne.setPosition(0);
                    platformServoTwo.setPosition(1);
                    stopwatch(4);
                }
                if(gamepad2.left_bumper && (capstonePlaced && System.currentTimeMillis()-milliTime[4] > timeThreshold)){
                    capstonePlaced = false;
                    capstoneServo.setPosition(1);
                    stopwatch(5);
                }
                if(gamepad2.left_bumper && (!capstonePlaced && System.currentTimeMillis()-milliTime[4] > timeThreshold)){
                    capstonePlaced = true;
                    capstoneServo.setPosition(0);
                    stopwatch(5);
                }
                if(gamepad1.left_bumper && (hyperDrive && System.currentTimeMillis()-milliTime[2] > timeThreshold)){
                    hyperDrive = false;
                    speed = .5;
                    stopwatch(3);
                }
                if(gamepad1.left_bumper && (!hyperDrive && System.currentTimeMillis()-milliTime[2] > timeThreshold)) {
                    hyperDrive = true;
                    speed = 1.0;
                    stopwatch(3);
                }
                telemetry.addData("Pos", "Slide: " + slideMotor.getCurrentPosition());
                telemetry.addData("Pos", "Servos " + platformServoOne.getPosition());
                telemetry.addData("Pos", "Servos2 " + platformServoTwo.getPosition());
                telemetry.update();
            }
    }
    private void stopwatch(int type){
        if(type == 1)
            milliTime[0] = System.currentTimeMillis();
        if(type == 2)
            milliTime[1] = System.currentTimeMillis();
        if(type == 3)
            milliTime[2] = System.currentTimeMillis();
        if(type == 4)
            milliTime[3] = System.currentTimeMillis();
        if(type == 5)
            milliTime[4] = System.currentTimeMillis();
        if(type == 6)
            milliTime[5] = System.currentTimeMillis();
    }
}
