package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    protected HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Servo flap, intakeStopper, flicker, claw;
    public DcMotor flywheelFront, flyWheelBack, intake;
    public boolean flywheelsRunning, intakeRunning, intakeReversed = false;
    public boolean clawClosed = true;
    double flapPosition;
    // public RobotPreferences pref;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drivetrain = new Drivetrain(hwMap);
        // pref = new RobotPreferences();
        initHardware();
        flapPosition = flap.getPosition();
    }

    /*
    Initializes the robot and the components
     */
    public void initHardware() {
        flap = this.hwMap.servo.get("flap");
        intakeStopper = this.hwMap.servo.get("intakeStopper");
        flicker = this.hwMap.servo.get("flicker");
        claw = this.hwMap.servo.get("claw");
        flywheelFront = this.hwMap.dcMotor.get("flywheelFront");
        flyWheelBack = this.hwMap.dcMotor.get("flywheelBack");
        intake = this.hwMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void startPositions() {
        flap.setPosition(0.5);
        flicker.setPosition(0.3);
        claw.setPosition(0.7);
    }

    public void setFlywheelPower(double power) {
        flywheelFront.setPower(power);
        flyWheelBack.setPower(power);
    }

    public void startFlywheels() {
        setFlywheelPower(1);
        flywheelsRunning = true;
    }

    public void stopFlywheels() {
        setFlywheelPower(0);
        flywheelsRunning = false;
    }

    public void startIntake() {
        setIntakePower(1);
        intakeRunning = true;
        intakeReversed = false;
    }

    public void stopIntake() {
        setIntakePower(0);
        intakeRunning = false;
        intakeReversed = false;
    }

    public void shootDisc() {
        flicker.setPosition(.6);
        sleep(50);
        flicker.setPosition(.3);
    }

    public void flapAdjustUp() {
        flap.setPosition(flapPosition - 0.01);
    }

    public void flapAdjustDown() {
        flap.setPosition(flapPosition + 0.01);
    }

    public void flapUpperPosition() {
        flap.setPosition(0.4);
    }

    public void flapLowerPosition() {
        flap.setPosition(0.6);
    }

    public void openClaw() {
        claw.setPosition(1);
        clawClosed = false;
    }

    public void closeClaw() {
        claw.setPosition(0.7);
        clawClosed = true;
    }

    public void toggleClaw() {
        if (clawClosed)
            openClaw();
        else closeClaw();
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void toggleIntake() {
        if (intakeRunning)
            stopIntake();
        else startIntake();
    }

    public void toggleFlywheels() {
        if (flywheelsRunning)
            stopFlywheels();
        else startFlywheels();
    }

    public void reverseIntake() {
        setIntakePower(-1);
        intakeReversed = true;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
