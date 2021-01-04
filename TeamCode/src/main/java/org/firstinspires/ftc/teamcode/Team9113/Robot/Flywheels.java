package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Flywheels {
    public Motor flywheelFront, flywheelBack;
    private boolean running = false, atMaxVelocity = false;
    public static double targetVelocity = 0;
    public static double maxVelocity = 1900;
    public static double power = 0;
    public static double kP = 12, kI = 12, kD = 0.3;
    private HardwareMap hwMap;

    public Flywheels(HardwareMap hwMap) {
        this.hwMap = hwMap;
        flywheelFront = new Motor(hwMap, "flywheelFront", Motor.GoBILDA.BARE);
        flywheelFront.setRunMode(Motor.RunMode.VelocityControl);
        flywheelFront.setVeloCoefficients(kP, kI, kD);
        flywheelBack = new Motor(hwMap, "flywheelBack", Motor.GoBILDA.BARE);
    }

    public void run() {
        setPower(targetVelocity / 2800);
        running = targetVelocity >= .01;
        atMaxVelocity = doubleEquals(targetVelocity, maxVelocity);
    }

    public void halt() {
        targetVelocity = 0;
        atMaxVelocity = doubleEquals(targetVelocity, maxVelocity);
        setPower(0);
        running = false;
    }

    public void start() {
        setPower(maxVelocity / 2800);
        running = true;
    }

    public void setPower(double power) {
        Flywheels.power = power;
        flywheelFront.set(power);
        flywheelBack.set(flywheelFront.get());
        running = power >= .01;
    }

    public void stop() {
        flywheelFront.set(0);
        flywheelBack.set(0);
        running = false;
    }

    public boolean running() {
        return running;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getActualVelocity() {
        return flywheelFront.get();
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void doMaxVelocity() {
        targetVelocity = maxVelocity;
    }

    public double getPower() {
        return power;
    }

    public void toggle() {
        if (running)
            stop();
        else start();
    }

    public boolean atMaxVelocity() {
        return atMaxVelocity;
    }

    public void togglePID() {
        if (atMaxVelocity())
            halt();
        else doMaxVelocity();
    }

    private static boolean doubleEquals(double a, double b) {
        if (a == b) return true;
        return Math.abs(a - b) < 0.0000001;
    }
}
