package org.firstinspires.ftc.teamcode.NFS.Control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is used as a gamepad-like tool for the teleop, where the teleop uses methods in the omnipad to determine whether a certain action should be done
 */
@Config
public class Omnipad {

    /**
     * The default button cooldown  
     */
    public static int cooldown = 350;

    /**
     * The ring shoot cooldown
     */
    public static int shootCooldown = 250;

    /**
     * The claw cooldown
     */
    public static int clawCooldown = 300;

    /**
     * The flap cooldown
     */
    public static int flapCooldown = 450;
    private final Robot robot;
    private final Gamepad one;
    private Gamepad two;
    private final double[] timer = new double[10];

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
     *
     * @return true if an arm up is requested
     */
    public boolean armUp() {
        return evalBoolean(one.y, 0);
    }

    /**
     * One/Y
     *
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
     *
     * @return true if a claw open is requested
     */
    public boolean clawOpen() {
        return evalBoolean(one.b, 1, clawCooldown);
    }

    /**
     * One/B
     *
     * @return true if a claw close is requested
     */
    public boolean clawClose() {
        return evalBoolean(one.b, 1, clawCooldown);
    }

    /**
     * One/B
     *
     * @return true if a claw toggle is requested
     */
    public boolean clawToggle() {
        return evalBoolean(one.b && !robot.arm.isUp(), 1, clawCooldown);
    }

    /**
     * One/A
     *
     * @return true if an intake start is requested
     */
    public boolean intakeStart() {
        return evalBoolean(one.a, 2);
    }

    /**
     * One/A
     *
     * @return true if an intake stop is requested
     */
    public boolean intakeStop() {
        return evalBoolean(one.a, 2);
    }

    /**
     * One/A
     *
     * @return true if an intake toggle is requested
     */
    public boolean intakeToggle() {
        return evalBoolean(one.a, 2, 500);
    }

    /**
     * One/Dpad Down
     *
     * @return true if a flap lower is requested
     */
    public boolean lowerFlap() {
        return evalBoolean(one.dpad_down, 3, flapCooldown);
    }

    /**
     * One/Dpad Up
     *
     * @return true if a flap raise is requested
     */
    public boolean raiseFlap() {
        return evalBoolean(one.dpad_up, 4, flapCooldown);
    }

    /**
     * One/Back
     *
     * @return true if a new offset should be set
     */
    public boolean setOffset() {
        return one.back;
    }

    /**
     * One/Left Bumper
     *
     * @return true if a flywheel start is requested
     */
    public boolean flywheelsStart() {
        return evalBoolean(one.left_bumper, 5);
    }

    /**
     * One/Left Bumper
     *
     * @return true if a flywheel stop is requested
     */
    public boolean flywheelsStop() {
        return evalBoolean(one.left_bumper, 5);
    }

    /**
     * One/Left Bumper
     *
     * @return true if a flywheel toggle is requested
     */
    public boolean flywheelsToggle() {
        return evalBoolean(one.left_bumper, 5);
    }

    /**
     * One/Right Bumper
     *
     * @return true if a ring shoot is requested
     */
    public boolean shootRing() {
        return evalBoolean(one.right_bumper, 6, shootCooldown);
    }

    /**
     * One/A
     * @return true if a ring shoot without an FSM is requested
     */
    public boolean shootRingNonFSM() {
        return evalBoolean(one.a, 8, shootCooldown);
    }

    /**
     * @return true if the drivetrain controls are inactive
     */
    public boolean drivetrainDormant() {
        return (one.right_stick_x <= 0.01 && one.right_stick_y <= 0.01) && (one.left_stick_y <= 0.01 && one.left_stick_x <= 0.01);
    }

    /**
     * One/X
     * @return true if an intake reversal is requested
     */
    public boolean intakeReverse() {
        return one.x;
    }

    /**
     * One/A
     * @return true if an intake stopper down is requested
     */
    public boolean intakeStopperDown(){
        return evalBoolean(one.a, 9);
    }

    /**
     * One/A
     * @return true if an intake stopper up is requested
     */
    public boolean intakeStopperUp(){
        return evalBoolean(one.a, 9);
    }

    /**
     * One/A
     * @return true if an intake stopper toggle is requested
     */
    public boolean intakeStopperToggle(){
        return evalBoolean(one.a, 9);
    }

    /**
     * One/Dpad Right
     *
     * @return true if a turn right is requested
     */
    public boolean turnRight() {
        return evalBoolean(one.dpad_right, 7);
    }

    /**
     * One/Dpad Left
     *
     * @return true if a turn left is requested
     */
    public boolean turnLeft() {
        return evalBoolean(one.dpad_left, 7);
    }

    /**
     * @return the value of the left stick y
     */
    public double getLeftY() {
        if(Math.abs(one.left_stick_y)> .05) return -one.left_stick_y * robot.drivetrain.getThrottle();
        else return 0;
    }

    /**
     * @return the value of the left stick x
     */
    public double getLeftX() {
        if (Math.abs(one.left_stick_x) > .05) return one.left_stick_x * robot.drivetrain.getThrottle();
        else return 0;
    }

    /**
     * @return the value of the right stick x
     */
    public double getRightX() {
        if(Math.abs(one.right_stick_x)> .05) return Math.sqrt(Math.abs(one.right_stick_x)) * one.right_stick_x * robot.drivetrain.getTurnThrottle();
        else return 0;
    }

    /**
     * Gives a Y value based on field centric values
     *
     * @param speed   speed of robot dictated by controls
     * @param heading heading given by the imu
     * @param ly      the value of left stick y
     * @param lx      the value of left stick x
     * @return the Y value to be used in field centric code
     */
    public double getY(double speed, double heading, double ly, double lx) {
        return speed * Math.sin(Math.atan2(ly, lx) - heading);
    }

    /**
     * Gives a X value based on field centric values
     *
     * @param speed   speed of robot dictated by controls
     * @param heading heading given by the imu
     * @param ly      the value of left stick y
     * @param lx      the value of left stick x
     * @return the X value to be used in field centric code
     */
    public double getX(double speed, double heading, double ly, double lx) {
        return speed * Math.cos(Math.atan2(ly, lx) - heading);
    }

    /**
     * Returns true if the given value and cooldown are valid
     *
     * @param value     the boolean value (typically a button)
     * @param stopwatch the index of the stopwatch for the action
     * @return true if the action should be committed
     */
    private boolean evalBoolean(boolean value, int stopwatch) {
        if (!value) return false;
        boolean temp = System.currentTimeMillis() - timer[stopwatch] >= cooldown;
        if (temp) stopwatch(stopwatch);
        return temp;
    }

    /**
     * Returns true if the given value and cooldown are valid
     *
     * @param value     the boolean value (typically a button)
     * @param stopwatch the index of the stopwatch for the action
     * @param cooldown  a specified cooldown value
     * @return true if the action should be committed
     */
    private boolean evalBoolean(boolean value, int stopwatch, int cooldown) {
        if (!value) return false;
        boolean temp = System.currentTimeMillis() - timer[stopwatch] >= cooldown;
        if (temp) stopwatch(stopwatch);
        return temp;
    }

    /**
     * A stopwatch to keep time of the initial times for button presses
     *
     * @param type the index of the stopwatch
     */
    private void stopwatch(int type) {
        timer[type] = System.currentTimeMillis();
    }
}
