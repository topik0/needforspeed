package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    protected HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Servo flap, intakeStopper, flicker, claw;
    public DcMotor intake, wobble;
    public DcMotorEx flywheelFront, flywheelBack;
    public boolean flywheelsRunning, intakeRunning, intakeReversed, flywheelsSlow = false;
    public boolean clawClosed = true, wobbleUp = true;
    double flapPosition;
    double flapHighGoal = .410, flapPowerShot = .7;
    double clawClosePosition = .705;
    // public RobotPreferences pref;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drivetrain = new Drivetrain(hwMap);
        // pref = new RobotPreferences();
        initHardware();
        flapPosition = flap.getPosition();
    }

    public Robot(HardwareMap hwMap, boolean passive) {
        this.hwMap = hwMap;
        flapPosition = flap.getPosition();
    }

    /*
    Initializes the robot and the components
     */
    public void initHardware() {
        flap = hwMap.servo.get("flap");
        intakeStopper = hwMap.servo.get("intakeStopper");
        flicker = hwMap.servo.get("flicker");
        claw = hwMap.servo.get("claw");
        flywheelFront = hwMap.get(DcMotorEx.class, "flywheelFront");
        flywheelBack = hwMap.get(DcMotorEx.class, "flywheelBack");
        intake = hwMap.dcMotor.get("intake");
        wobble = hwMap.dcMotor.get("wobble");
        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startPositions() {
        flap.setPosition(flapHighGoal);
        flicker.setPosition(0.3);
        claw.setPosition(clawClosePosition);
    }

    public void startPositions(boolean isAuto) {
        flap.setPosition(.3);
        flicker.setPosition(0.3);
        claw.setPosition(clawClosePosition);
    }

    public void wobbleUp() {
        wobble.setPower(-1);
        sleep(300);
        wobble.setPower(0);
        wobbleUp = true;
    }

    public void wobbleDown() {
        wobble.setPower(0.6);
        sleep(230);
        wobble.setPower(0);
        wobbleUp = false;
    }

    public void intakeDown() {
        intake.setPower(.2);
        sleep(250);
        stopIntake();
    }

    public void toggleWobble() {
        if (wobbleUp)
            wobbleDown();
        else wobbleUp();
    }

    public void setFlywheelPower(double power) {
        flywheelFront.setPower(power);
        flywheelBack.setPower(power);
    }

    public void setFlywheelVelocity(double velocity) {
        flywheelFront.setVelocity(velocity);
        flywheelBack.setVelocity(velocity);
    }

    public void startFlywheels() {
        setFlywheelVelocity(2350);
        flywheelsRunning = true;
    }

    public void startFlyWheelsSlow() {
        setFlywheelVelocity(1500);
        flywheelsRunning = true;
    }

    public void stopFlywheels() {
        setFlywheelVelocity(0);
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
        flicker.setPosition(.55);
        sleep(60);
        flicker.setPosition(.3);
    }

    public void flapAdjustUp() {
        flapPosition = flap.getPosition();
        flap.setPosition(flapPosition - .005);
        sleep(50);
    }

    public void flapAdjustDown() {
        flapPosition = flap.getPosition();
        flap.setPosition(flapPosition + .005);
        sleep(50);
    }

    public void flapUpperPosition() {
        flap.setPosition(flapHighGoal);
        sleep(100);
        flapPosition = flap.getPosition();
    }

    public void flapLowerPosition() {
        flap.setPosition(flapPowerShot);
        sleep(100);
        flapPosition = flap.getPosition();
    }

    public void openClaw() {
        claw.setPosition(1);
        clawClosed = false;
        sleep(350);
    }

    public void closeClaw() {
        claw.setPosition(clawClosePosition);
        sleep(350);
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
        else
            startIntake();
    }

    public void toggleFlywheels() {
        if (flywheelsRunning)
            stopFlywheels();
        else startFlywheels();
    }

    public void toggleFlywheelsMode() {
        if (flywheelsSlow)
            startFlyWheelsSlow();
        else
            startFlywheels();
    }

    public void reverseIntake() {
        setIntakePower(-1);
        intakeReversed = true;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}