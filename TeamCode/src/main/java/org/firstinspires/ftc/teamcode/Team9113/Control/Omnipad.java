package org.firstinspires.ftc.teamcode.Team9113.Control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Team9113.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;

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

    public Omnipad(Gamepad one, Gamepad two, Robot robot) {
        this.one = one;
        this.two = two;
        this.robot = robot;
    }

    public boolean armUp() {
        return evalBoolean(one.y, 0);
    }

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

    public boolean clawOpen() {
        return evalBoolean(one.b, 1, clawCooldown);
    }

    public boolean clawClose() {
        return evalBoolean(one.b, 1, clawCooldown);
    }

    public boolean clawToggle() {
        return evalBoolean(one.b && !robot.arm.isUp(), 1, clawCooldown);
    }

    public boolean intakeStart() {
        return evalBoolean(one.a, 2);
    }

    public boolean intakeStop() {
        return evalBoolean(one.a, 2);
    }

    public boolean intakeToggle() {
        return evalBoolean(one.a, 2, 500);
    }

    public boolean lowerFlap() {
        return evalBoolean(one.dpad_down, 3, flapCooldown);
    }

    public boolean raiseFlap() {
        return evalBoolean(one.dpad_up, 4, flapCooldown);
    }

    public boolean setOffset() {
        return one.back;
    }

    public boolean flywheelsStart() {
        return evalBoolean(one.left_bumper, 5);
    }

    public boolean flywheelsStop() {
        return evalBoolean(one.left_bumper, 5);
    }

    public boolean flywheelsToggle() {
        return evalBoolean(one.left_bumper, 5);
    }

    public boolean shootRing() {
        return evalBoolean(one.right_bumper, 6, shootCooldown);
    }

    public boolean drivetrainDormant() {
        return (one.right_stick_x <= .01 && one.left_stick_x <= .01) && (one.left_stick_y <= 0.01 && one.left_stick_x <= .01);
    }

    public boolean intakeReverse() {
        return one.x;
    }

    public boolean turnRight() {
        return evalBoolean(one.dpad_right, 7);
    }

    public boolean turnLeft() {
        return evalBoolean(one.dpad_right, 7);
    }

    public double getLeftY() {
        return -one.left_stick_y * Drivetrain.currentThrottle;
    }

    public double getLeftX() {
        return one.left_stick_y * Drivetrain.currentThrottle;
    }

    public double getRightX() {
        return -1 * Math.sqrt(Math.abs(one.right_stick_x)) * one.right_stick_x * Drivetrain.turnThrottle;
    }

    public double getY(double speed, double heading, double ly, double lx) {
        return speed * Math.sin(Math.atan2(ly, lx) - heading);
    }

    public double getX(double speed, double heading, double ly, double lx) {
        return speed * Math.cos(Math.atan2(ly, lx) - heading);
    }

    public boolean evalBoolean(boolean value, int stopwatch) {
        if (!value) return false;
        boolean temp = System.currentTimeMillis() - timer[stopwatch] > cooldown;
        if (temp) stopwatch(stopwatch);
        return temp;
    }

    public boolean evalBoolean(boolean value, int stopwatch, int cooldown) {
        if (!value) return false;
        boolean temp = System.currentTimeMillis() - timer[stopwatch] > cooldown;
        if (temp) stopwatch(stopwatch);
        return temp;
    }

    public void stopwatch(int type) {
        timer[type] = System.currentTimeMillis();
    }
}
