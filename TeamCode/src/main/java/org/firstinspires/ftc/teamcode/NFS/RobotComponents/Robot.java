package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.time.StopWatch;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class has all of the robot functions and classes, including all robot objects, Hardware Maps, and PID delays
 */
@Config
public class Robot {
    /**
     * The hardware map used to make HardwareGenesis
     */
    public HardwareMap hwMap;
    /**
     * HardwareGenesis to pass on to robot components
     */
    public HardwareGenesis genesis;
    /**
     * Robot drivetrain
     */
    public Drivetrain drivetrain;
    /**
     * Robot flywheels
     */
    public Flywheels flywheels;
    /**
     * Robot flicker (to shoot)
     */
    public Flicker flicker;
    /**
     * The wobble arm
     */
    public Arm arm;
    /**
     * The flap to aim rings
     */
    public Flap flap;
    /**
     * The claw mechanism to grab the wobble goal
     */
    public Claw claw;
    /**
     * The robot intake
     */
    public Intake intake;
    /**
     * OP Mode telemetry
     */
    public Telemetry telemetry;
    private final Mode mode;

    /**
     * An enum to keep track of whether the robot is being made in a TeleOP or Autonomous program
     */
    private enum Mode {
        TELEOP,
        AUTO
    }

    /**
     * Robot constructor for teleop.  This creates all of the hardware objects and sets the mode to teleop
     *
     * @param hwMap the hardware map of the op mode from which it was made
     */
    public Robot(HardwareMap hwMap, Telemetry telemetry) {
        mode = Mode.TELEOP;
        this.telemetry = telemetry;
        this.hwMap = hwMap;
        genesis = new HardwareGenesis(hwMap);
        arm = new Arm(genesis);
        flap = new Flap(genesis, this);
        claw = new Claw(genesis);
        intake = new Intake(genesis);
        drivetrain = new Drivetrain(genesis, this);
        flywheels = new Flywheels(genesis, this);
        flicker = new Flicker(genesis, this);
    }

    /**
     * Robot constructor for auto.  This creates all of the hardware objects and sets the mode to auto
     *
     * @param hwMap the hardware map of the op mode from which it was made
     */
    public Robot(HardwareMap hwMap, Telemetry telemetry, boolean isAuto) {
        if (isAuto) mode = Mode.AUTO;
        else mode = Mode.TELEOP;
        this.telemetry = telemetry;
        this.hwMap = hwMap;
        genesis = new HardwareGenesis(hwMap);
        arm = new Arm(genesis);
        flap = new Flap(genesis, this);
        claw = new Claw(genesis);
        intake = new Intake(genesis);
        drivetrain = new Drivetrain(genesis, this);
        flywheels = new Flywheels(genesis, this);
        flicker = new Flicker(genesis, this);
        Flicker.noShootWhileFlywheelsDormant = false;
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

    /**
     * Checks if the robot is in autonomous mode
     *
     * @return true if the robot is in autonomous mode
     */
    public boolean isAuto() {
        return mode == Mode.AUTO;
    }
}