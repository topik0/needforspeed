package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.lang3.time.StopWatch;

import java.util.concurrent.TimeUnit;

@Config
public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    private HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Flywheels flywheels;
    public Flicker flicker;
    public Servo flap, claw, wobble, intakeStopper;
    public DcMotor leftIntake, rightIntake;
    public boolean intakeRunning, intakeReversed;
    public boolean clawClosed = true, wobbleUp = true;
    double flapPosition;
    public static double flapHighGoal = .2785, flapPowerShot = .3;
    public static int shootDelay = 80;
    public static double clawClosePosition = .42;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drivetrain = new Drivetrain(hwMap);
        flywheels = new Flywheels(hwMap);
        flicker = new Flicker(hwMap, drivetrain.mecanumDrive);
        // pref = new RobotPreferences();
        initHardware();
        flapPosition = flap.getPosition();
    }

    public Robot(HardwareMap hwMap, boolean noDrivetrain) {
        this.hwMap = hwMap;
        initHardware();
        flapPosition = flap.getPosition();
    }

    /*
    Initializes the robot and the components
     */
    public void initHardware() {
        flap = hwMap.servo.get("flap");
        claw = hwMap.servo.get("claw");
        wobble = hwMap.servo.get("wobble");
        intakeStopper = hwMap.servo.get("intakeStopper");
        leftIntake = hwMap.dcMotor.get("leftIntake");
        rightIntake = hwMap.dcMotor.get("rightIntake");
        //flywheelFront.setVeloCoefficients(1.27272727273, 0, 0.01);
        // pcont = new PController(1);
        //pcont = new PIDController(5, 0, 0 );
    }

    public void startPositions() {
        flap.setPosition(flapHighGoal);
        wobbleUp();
        flicker.startPosition();
        claw.setPosition(clawClosePosition);
        intakeStopper.setPosition(.85);
    }

    public void startPositions(boolean isAuto) {
        flap.setPosition(.1);
        flicker.startPosition();
        claw.setPosition(clawClosePosition);
        intakeStopper.setPosition(.85);
        wobbleUp();
    }

    public void wobbleUp() {
        wobble.setPosition(.69);
        wobbleUp = true;
    }

    public void wobbleDown() {
        wobble.setPosition(.07);
        wobbleUp = false;
    }

    public void intakeDown() {
        intakeStopper.setPosition(1.0);
    }

    public void toggleWobble() {
        if (wobbleUp)
            wobbleDown();
        else wobbleUp();
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

    public void flapAdjustUp() {
        flapPosition = flap.getPosition();
        flap.setPosition(flapPosition - .005);
    }

    public void flapAdjustDown() {
        flapPosition = flap.getPosition();
        flap.setPosition(flapPosition + .005);
    }

    public void flapUpperPosition() {
        flap.setPosition(flapHighGoal);
        flapPosition = flap.getPosition();
    }

    public void flapLowerPosition() {
        flap.setPosition(flapPowerShot);
        flapPosition = flap.getPosition();
    }

    public void openClaw() {
        claw.setPosition(.74);
        clawClosed = false;
    }

    public void closeClaw() {
        claw.setPosition(clawClosePosition);
        clawClosed = true;
    }

    public void toggleClaw() {
        if (clawClosed)
            openClaw();
        else closeClaw();
    }

    public void setIntakePower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void toggleIntake() {
        if (intakeRunning)
            stopIntake();
        else
            startIntake();
    }

    public void reverseIntake() {
        setIntakePower(-1);
        intakeReversed = true;
    }

    public void delayWithFlywheelPID(double milliseconds) {
        StopWatch stopwatch = new StopWatch();
        stopwatch.start();
        while (stopwatch.getTime() <= milliseconds)
            flywheels.run();
    }

    public void delayWithAllPID(double milliseconds) {
        StopWatch stopwatch = new StopWatch();
        stopwatch.start();
        while (stopwatch.getTime() <= milliseconds)
            drivetrain.mecanumDrive.update();
    }

    /*
    Turns the robot a specified angle measured in radians
     */
    public void turn(double angle) {
        drivetrain.mecanumDrive.turn(angle);
    }

    @Override
    public void runOpMode() {
    }
}