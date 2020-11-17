package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    protected HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Servo flap, intakeStopper, flicker, claw;
    public DcMotor flywheelFront, flyWheelBack, intake, wobble;
    public boolean flywheelsRunning, intakeRunning, intakeReversed = false;
    public boolean clawClosed = true, wobbleUp = true;
    double flapPosition;
    double flapHighGoal = .4, flapPowerShot = .44;
    double clawClosePosition = .725;
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
        flap = this.hwMap.servo.get("flap");
        intakeStopper = this.hwMap.servo.get("intakeStopper");
        flicker = this.hwMap.servo.get("flicker");
        claw = this.hwMap.servo.get("claw");
        flywheelFront = this.hwMap.dcMotor.get("flywheelFront");
        flyWheelBack = this.hwMap.dcMotor.get("flywheelBack");
        intake = this.hwMap.dcMotor.get("intake");
        wobble = this.hwMap.dcMotor.get("wobble");
        intake.setDirection(DcMotor.Direction.REVERSE);
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
        wobble.setPower(-0.7);
        sleep(550);
        wobble.setPower(0);
        wobbleUp = true;
    }

    public void wobbleDown() {
        wobble.setPower(0.6);
        sleep(300);
        wobble.setPower(0);
        wobbleUp = false;
    }

    public void toggleWobble() {
        if (wobbleUp)
            wobbleDown();
        else wobbleUp();
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
