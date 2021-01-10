package org.firstinspires.ftc.teamcode.Team9113.Control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Team9113.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * <p>
 * This class is used as a gamepad-like tool for the teleop, where the teleop uses methods in the omnipad to determine whether a certain action should be done
 */
@Config
public class Omnipad {
    private Robot robot;
    private Gamepad one;
    private Gamepad two;
    private double[] timer = new double[9];
    public static double cooldown = 350;
    public static int shootCooldown = 80;
    public static int clawCooldown = 700;
    public static int flapCooldown = 450;

    /**
     * Constructor for the omnipad
     *
     * @param one   gamepad one
     * @param two   gamepad two
     * @param robot the robot that contains robot objects
     */
    public Omnipad(Gamepad one, Gamepad two, Robot robot) {
        this.one = one;
        this.two = two;
        this.robot = robot;
    }

    /**
     * One/Y
     * @return true if an arm up is requested
     */
    public boolean armUp() {
        return evalBoolean(one.y, 0);
    }

    /**
     * One/Y
     * @return true if an arm down is requested
     */
    public boolean armDown() {
        return evalBoolean(one.y, 0);
    }

    /**
     * One/Y
     *
     * @return true if an arm toggle is requested
     */
    public boolean armToggle() {
        return evalBoolean(one.y, 0);
    }

    /**
     * One/B
     * @return true if a claw open is requested
     */
    public boolean clawOpen() {
        return evalBoolean(one.b, 1, clawCooldown);
    }

    /**
     * One/B
     * @return true if a claw close is requested
     */
    public boolean clawClose() {
        return evalBoolean(one.b, 1, clawCooldown);
    }

    /**
     * One/B
     * @return true if a claw toggle is requested
     */
    public boolean clawToggle() {
        return evalBoolean(one.b && !robot.arm.isUp(), 1, clawCooldown);
    }

    /**
     * One/A
     * @return true if an intake start is requested
     */
    public boolean intakeStart() {
        return evalBoolean(one.a, 2);
    }

    /**
     * One/A
     * @return true if an intake stop is requested
     */
    public boolean intakeStop() {
        return evalBoolean(one.a, 2);
    }

    /**
     * One/A
     * @return true if an intake toggle is requested
     */
    public boolean intakeToggle() {
        return evalBoolean(one.a, 2, 500);
    }

    /**
     * One/Dpad Down
     * @return true if a flap lower is requested
     */
    public boolean lowerFlap() {
        return evalBoolean(one.dpad_down, 3, flapCooldown);
    }

    /**
     * One/Dpad Up
     * @return true if a flap raise is requested
     */
    public boolean raiseFlap() {
        return evalBoolean(one.dpad_up, 4, flapCooldown);
    }

    /**
     * One/Back
     * @return true if a new offset should be set
     */
    public boolean setOffset() {
        return one.back;
    }

    /**
     * One/Left Bumper
     * @return true if a flywheel start is requested
     */
    public boolean flywheelsStart() {
        return evalBoolean(one.left_bumper, 5);
    }

    /**
     * One/Left Bumper
     * @return true if a flywheel stop is requested
     */
    public boolean flywheelsStop() {
        return evalBoolean(one.left_bumper, 5);
    }

    /**
     * One/Left Bumper
     * @return true if a flywheel toggle is requested
     */
    public boolean flywheelsToggle() {
        return evalBoolean(one.left_bumper, 5);
    }

    /**
     * One/Right Bumper
     * @return true if a ring shoot is requested
     */
    public boolean shootRing() {
        return evalBoolean(one.right_bumper, 6, shootCooldown);
    }

    /**
     * @return true if the drivetrain controls are inactive
     */
    public boolean drivetrainDormant() {
        return (one.right_stick_x <= .01 && one.left_stick_x <= .01) && (one.left_stick_y <= 0.01 && one.left_stick_x <= .01);
    }

    /**
     * One/X
     * @return true if an intake reversal is requested
     */
    public boolean intakeReverse() {
        return one.x;
    }

    /**
     * One/Dpad Right
     * @return true if a turn right is requested
     */
    public boolean turnRight() {
        return evalBoolean(one.dpad_right, 7);
    }

    /**
     * One/Dpad Left
     * @return true if a turn left is requested
     */
    public boolean turnLeft() {
        return evalBoolean(one.dpad_left, 7);
    }

    /**
     * @return the value of the left stick y
     */
    public double getLeftY() {
        return -one.left_stick_y * Drivetrain.currentThrottle;
    }

    /**
     * @return the value of the left stick x
     */
    public double getLeftX() {
        return one.left_stick_x * Drivetrain.currentThrottle;
    }

    /**
     * @return the value of the right stick x
     */
    public double getRightX() {
        return -1 * Math.sqrt(Math.abs(one.right_stick_x)) * one.right_stick_x * Drivetrain.turnThrottle;
    }

    /**
     * Gives a Y value based on field centric values
     * @param speed speed of robot dictated by controls
     * @param heading heading given by the imu
     * @param ly the value of left stick y
     * @param lx the value of left stick x
     * @return the Y value to be used in field centric code
     */
    public double getY(double speed, double heading, double ly, double lx) {
        return speed * Math.sin(Math.atan2(ly, lx) - heading);
    }
    /**
     * Gives a X value based on field centric values
     * @param speed speed of robot dictated by controls
     * @param heading heading given by the imu
     * @param ly the value of left stick y
     * @param lx the value of left stick x
     * @return the X value to be used in field centric code
     */
    public double getX(double speed, double heading, double ly, double lx) {
        return speed * Math.cos(Math.atan2(ly, lx) - heading);
    }

    /**
     * Returns true if the given value and cooldown are valid
     * @param value the boolean value (typically a button)
     * @param stopwatch the index of the stopwatch for the action
     * @return true if the action should be committed
     */
    private boolean evalBoolean(boolean value, int stopwatch) {
        if (!value) return false;
        boolean temp = System.currentTimeMillis() - timer[stopwatch] > cooldown;
        if (temp) stopwatch(stopwatch);
        return temp;
    }

    /**
     * Returns true if the given value and cooldown are valid
     * @param value the boolean value (typically a button)
     * @param stopwatch the index of the stopwatch for the action
     * @param cooldown a specified cooldown value
     * @return true if the action should be committed
     */
    private boolean evalBoolean(boolean value, int stopwatch, int cooldown) {
        if (!value) return false;
        boolean temp = System.currentTimeMillis() - timer[stopwatch] > cooldown;
        if (temp) stopwatch(stopwatch);
        return temp;
    }

    /**
     * A stopwatch to keep time of the initial times for button presses
     * @param type the index of the stopwatch
     */
    private void stopwatch(int type) {
        timer[type] = System.currentTimeMillis();
    }
}
