package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.lang3.time.StopWatch;

@Config
public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    public HardwareMap hwMap;
    public HardwareGenesis genesis;
    public Drivetrain drivetrain;
    public Flywheels flywheels;
    public Flicker flicker;
    public Arm arm;
    public Flap flap;
    public Claw claw;
    public Intake intake;
    public Servo intakeStopper;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        genesis = new HardwareGenesis(hwMap);
        arm = new Arm(genesis);
        flap = new Flap(genesis);
        claw = new Claw(genesis);
        intake = new Intake(genesis);
        drivetrain = new Drivetrain(genesis);
        flywheels = new Flywheels(genesis);
        flicker = new Flicker(genesis);
    }

    public void startPositions() {
        flap.setStartPosition();
        arm.setStartPosition();
        flicker.startPosition();
        claw.setStartPosition();
        intakeStopper.setPosition(.85);
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