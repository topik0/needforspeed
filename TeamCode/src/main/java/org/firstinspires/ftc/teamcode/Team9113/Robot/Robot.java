package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.time.StopWatch;


/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * <p>
 * This class all of the robot functions and classes, including all robot objects, Hardware Maps, and PID delays
 */
@Config
public class Robot {
    public HardwareMap hwMap;
    public HardwareGenesis genesis;
    public Drivetrain drivetrain;
    public Flywheels flywheels;
    public Flicker flicker;
    public Arm arm;
    public Flap flap;
    public Claw claw;
    public Intake intake;

    /**
     * Robot constructor.  This creates all of the hardware objects
     *
     * @param hwMap the hardware map of the op mode from which it was made
     */
    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        genesis = new HardwareGenesis(hwMap);
        arm = new Arm(genesis);
        flap = new Flap(genesis, this);
        claw = new Claw(genesis);
        intake = new Intake(genesis);
        drivetrain = new Drivetrain(genesis);
        flywheels = new Flywheels(genesis);
        flicker = new Flicker(genesis, drivetrain.mecanumDrive, this);
    }

    /**
     * Moves all Robot objects to their respective starting positions
     */
    public void startPositions() {
        flap.setStartPosition();
        arm.setStartPosition();
        flicker.startPosition();
        claw.setStartPosition();
        intake.stopperStartPosition();
    }

    /**
     * Sets a delay while looping flywheel PID
     *
     * @param milliseconds the amount of delay time in milliseconds
     */
    public void delayWithFlywheelPID(double milliseconds) {
        StopWatch stopwatch = new StopWatch();
        stopwatch.start();
        while (stopwatch.getTime() <= milliseconds)
            flywheels.run();
    }

    /**
     * Sets a delay while looping all PID
     *
     * @param milliseconds the amount of delay time in milliseconds
     */
    public void delayWithAllPID(double milliseconds) {
        StopWatch stopwatch = new StopWatch();
        stopwatch.start();
        while (stopwatch.getTime() <= milliseconds)
            drivetrain.mecanumDrive.update();
    }
}