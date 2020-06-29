package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public DcMotor motorZero, motorOne, motorTwo, motorThree, motorFour, motorFive, slideMotorOne, slideMotorTwo;
    public Servo capstoneServo, jointServo, shoulderServo, platformServoOne, platformServoTwo, grabberArm, grabberElbow;
    public CRServo vex;

    private double speed = 1.0;
    public double strafeSpeed = 1.0;
    public boolean intakeIn, intakeStopped, hyperDriveEngaged = true;
    public boolean shoulderServoOut, clawClamped, capstonePlaced, platformServoDown, snailDriveEngaged = false;
    public int slideLevel = 0;
    private double liftPowOffsetThreshold = .05;
    private double liftPowOffset = .3;
    private double liftTicksPerInch = 303;
    private double maxLiftExtension = 38.5;
    private double originalDirection = 0;

    public Drivetrain drivetrain;
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    private Orientation lastAngles = new Orientation();
    /*
    Robot constructor
     */
    public Robot(){
        drivetrain = new Drivetrain();
        initHardware();
    }
    /*
    Initializes the robot and the components
     */
    public void initHardware(){

    }
    /*
    Moves the robot left a certain number of ticks at a given speed
     */
    public void moveLeft(int ticks, double speed) {
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorZero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorThree.setTargetPosition(-ticks);
        motorOne.setTargetPosition(ticks);
        motorTwo.setTargetPosition(ticks);
        motorZero.setTargetPosition(-ticks);

        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorZero.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorThree.setPower(-speed);
        motorTwo.setPower(speed);
        motorOne.setPower(speed);
        motorZero.setPower(-speed);
        while(motorZero.isBusy()) {
            telemetry.addData("", "Runtime: ", motorZero.getCurrentPosition());
            telemetry.update();
        }
    }
    /*
    Moves the robot right a certain number of ticks at a given speed
     */
    public void moveRight(int ticks, double speed) {
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorZero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorThree.setTargetPosition(ticks);
        motorOne.setTargetPosition(-ticks);
        motorTwo.setTargetPosition(-ticks);
        motorZero.setTargetPosition(ticks);

        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorZero.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorThree.setPower(speed);
        motorTwo.setPower(-speed);
        motorOne.setPower(-speed);
        motorZero.setPower(speed);
        while(motorZero.isBusy()) {
            telemetry.addData("", "Runtime: ", motorZero.getCurrentPosition());
            telemetry.update();
        }
    }
    /*
    Moves the robot forward a certain number of ticks at a given speed
     */
    public void moveForward(int ticks, double speed) {
        moveBackward(-ticks, speed);
    }
    /*
     Moves the robot backward a certain number of ticks at a given speed
     */
    public void moveBackward(int ticks, double speed) {
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorZero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorThree.setTargetPosition(ticks);
        motorOne.setTargetPosition(ticks);
        motorTwo.setTargetPosition(ticks);
        motorZero.setTargetPosition(ticks);

        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorZero.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorThree.setPower(speed);
        motorTwo.setPower(speed);
        motorOne.setPower(speed);
        motorZero.setPower(speed);
        while(motorZero.isBusy()) {
            telemetry.addData("", "Runtime: ", motorZero.getCurrentPosition());
            telemetry.update();
        }
    }
    /*
     Turns the robot left a certain number of ticks at a given speed
     */
    public void turnLeft(int ticks, double speed) {
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorZero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorThree.setTargetPosition(ticks);
        motorOne.setTargetPosition(ticks);
        motorTwo.setTargetPosition(-ticks);
        motorZero.setTargetPosition(-ticks);

        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorZero.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorThree.setPower(speed);
        motorTwo.setPower(-speed);
        motorOne.setPower(speed);
        motorZero.setPower(-speed);
        while(motorZero.isBusy()) {
            telemetry.addData("", "Runtime: ", motorZero.getCurrentPosition());
            telemetry.update();
        }
    }
    /*
     Turns the robot right a certain number of ticks at a given speed
     */
    public void turnRight(int ticks, double speed) {
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorZero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorThree.setTargetPosition(-ticks);
        motorOne.setTargetPosition(-ticks);
        motorTwo.setTargetPosition(ticks);
        motorZero.setTargetPosition(ticks);

        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorZero.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorThree.setPower(-speed);
        motorTwo.setPower(speed);
        motorOne.setPower(-speed);
        motorZero.setPower(speed);
        while(motorZero.isBusy()) {
            telemetry.addData("", "Runtime: ", motorZero.getCurrentPosition());
            telemetry.update();
        }
    }
    /*
    Brings the grabber arm down
     */
    public void grabberDown(){
        grabberArm.setPosition(.45);
    }
    /*
    Brings the grabber arm up
     */
    public void grabberUp(){
        grabberArm.setPosition(.12);
    }
    /*
    Brings the grabber elbow out
     */
    public void elbowOut(){
        grabberElbow.setPosition(.25);
    }
    /*
    Brings the grabber elbow in
     */
    public void elbowIn(){
        grabberElbow.setPosition(.9);
    }
    /*
    Bring the grabber arm and elbow to the place position (not tested)
     */
    public void grabberPlace(){
        grabberDown();
        elbowOut();
    }
    /*
     Bring the grabber arm and elbow to the grab position (not tested)
     */
    public void grabberGrab(){
        elbowIn();
        grabberUp();
    }
    /*
    Grabs a block using the grabber arm
     */
    public void grabberGrabMotionOne(){
        elbowOut();
        sleep(300);
        grabberDown();
        sleep(500);
        elbowIn();
        sleep(750);
        grabberUp();
    }
    /*
    Second grabber motion (semi-deprecated)
     */
    public void grabberGrabMotionTwo(){
        elbowOut();
        sleep(300);
        grabberDown();
        sleep(600);
        moveRight(650, .8);
        sleep(50);
        elbowIn();
        sleep(800);
        grabberUp();
    }
    /*
    Fix the robot heading by turning left or right to adjust.  In other words, it turns in a direction until the heading is 0
     */
    /*
    Places a block using the grabber arm
     */
    public void grabberPlaceMotion(){
        grabberDown();
        sleep(350);
        grabberElbow.setPosition(.5);
        sleep(500);
        grabberUp();
        elbowOut();
        sleep(750);
    }
    /*
    Makes the robot sleep (without any errors)
     */
    private void sleep(int milliseconds){
        sleep(milliseconds);
    }
    /*
    Brings the grabber elbow to the start position
     */
    public void grabberElbowStart(){ grabberElbow.setPosition(1); }
    /*
    Brings the grabber arm to the start position
     */
    public void grabberArmStart(){ grabberArm.setPosition(.18); }
    /*
    Brings everything to the start positions
     */
    public void startPositions(){
        platformServoOne.setPosition(.3);
        platformServoTwo.setPosition(.65);
        jointServo.setPosition(.15);
        capstoneServo.setPosition(1);
        shoulderServo.setPosition(.02);
        grabberArm.setPosition(.18);
        grabberElbow.setPosition(1);
    }
}
