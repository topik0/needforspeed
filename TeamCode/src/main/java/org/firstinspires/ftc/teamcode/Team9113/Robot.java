package org.firstinspires.ftc.teamcode.Team9113;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    protected HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Servo flap, flicker, claw, wobble;
    public DcMotor leftIntake, rightIntake;
    public Motor flywheelFront, flywheelBack;
    public boolean flywheelsRunning = false, intakeRunning, intakeReversed, flywheelsSlow = false;
    public boolean clawClosed = true, wobbleUp = true;
    double flapPosition;
    double flapHighGoal = .403, flapPowerShot = .43;
    double clawClosePosition = .74;
    double velo;
    //private double kI;
    //public static double kp;
   // public static double kd;
    //PIDController pcont;
    // public RobotPreferences pref;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drivetrain = new Drivetrain(hwMap);
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
        flicker = hwMap.servo.get("flicker");
        claw = hwMap.servo.get("claw");
        wobble = hwMap.servo.get("wobble");
        flywheelFront = new Motor(hwMap, "flywheelFront", Motor.GoBILDA.BARE);
        flywheelFront.setRunMode(Motor.RunMode.VelocityControl);
        flywheelFront.setVeloCoefficients(12, 0, 0);
        flywheelBack = new Motor(hwMap, "flywheelBack", Motor.GoBILDA.BARE);
        flywheelBack.setInverted(true);
        leftIntake = hwMap.dcMotor.get("leftIntake");
        rightIntake = hwMap.dcMotor.get("rightIntake");
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        flywheelFast();
        //flywheelFront.setVeloCoefficients(1.27272727273, 0, 0.01);
        // pcont = new PController(1);
        //pcont = new PIDController(5, 0, 0 );
    }

    public void startPositions() {
        flap.setPosition(flapHighGoal);
        wobbleUp();
        flicker.setPosition(0.3);
        claw.setPosition(clawClosePosition);
    }

    public void startPositions(boolean isAuto) {
        flap.setPosition(.3);
        flicker.setPosition(0.3);
        claw.setPosition(clawClosePosition);
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
        leftIntake.setPower(.2);
        // rightIntake.setPower(.2);
        delay(250);
        stopIntake();
    }

    public void toggleWobble() {
        if (wobbleUp)
            wobbleDown();
        else wobbleUp();
    }

   /* public void setFlywheelVelo(double targetVelo) {
        flywheelFront.setVeloCoefficients(1, 0, 0);
        double[] coeffs = flywheelFront.getVeloCoefficients();
        double kP = coeffs[0];
        double kI = coeffs[1];
        double kD = coeffs[2];

        flywheelBack.set(flywheelFront.get());
    } */

    public void flywheelFast() {
        velo = 1550  ;
        //flywheelsRunning = true;
    }

    public void flywheelSlow() {
        velo = 1550 ;
        //flywheelsRunning = true;
    }

    public void stopFlywheels() {
        flywheelFront.set(0);
        flywheelBack.set(0);
        //flywheelsRunning = false;
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
        sleep(70);
        flicker.setPosition(.3);
    }

    public void flapAdjustUp() {
        flapPosition = flap.getPosition();
        flap.setPosition(flapPosition - .005);
        delay(50);
    }

    public void flapAdjustDown() {
        flapPosition = flap.getPosition();
        flap.setPosition(flapPosition + .005);
        delay(50);
    }

    public void flapUpperPosition() {
        flap.setPosition(flapHighGoal);
        delay(100);
        flapPosition = flap.getPosition();
    }

    public void flapLowerPosition() {
        flap.setPosition(flapPowerShot);
        delay(100);
        flapPosition = flap.getPosition();
    }

    public void openClaw() {
        claw.setPosition(1);
        clawClosed = false;
        delay(350);
    }

    public void closeClaw() {
        claw.setPosition(clawClosePosition);
        delay(350);
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

    public void toggleFlywheels() {
        flywheelsRunning = !flywheelsRunning ;
    }

    public void toggleFlywheelsMode() {
        if (flywheelsSlow)
            flywheelSlow();
        else
            flywheelFast();
    }

    public void setFlywheelsModeSlow() {
        flywheelsSlow = true;
    }

    public void setFlywheelsModeNormal() {
        flywheelsSlow = false;
    }

    public void reverseIntake() {
        setIntakePower(-1);
        intakeReversed = true;
    }

    public void delay(int milliseconds){
        if(milliseconds < 0)
            throw new IllegalArgumentException("Cannot have a delay less than zero");
        if(milliseconds == 0)
            return;
        int timer = (int) System.currentTimeMillis();
        while(System.currentTimeMillis() - timer <= milliseconds);
    }

    public void turnAsync(double angle){
        drivetrain.mecanumDrive.turnAsync(angle);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}