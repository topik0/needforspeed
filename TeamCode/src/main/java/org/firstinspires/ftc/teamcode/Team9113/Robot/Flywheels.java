package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheels {
    public Motor flywheelFront, flywheelBack;
    private boolean flywheelsRunning = false;
    private double flywheelsTargetVelocity = 1600;
    public static double kP = 12, kI = 0, kD = 0;
    private HardwareMap hwMap;

    public Flywheels(HardwareMap hwMap) {
        this.hwMap = hwMap;
        flywheelFront = new Motor(hwMap, "flywheelFront", Motor.GoBILDA.BARE);
        flywheelFront.setRunMode(Motor.RunMode.VelocityControl);
        flywheelFront.setVeloCoefficients(kP, kI, kD);
        flywheelBack = new Motor(hwMap, "flywheelBack", Motor.GoBILDA.BARE);
    }

    public void start() {
        flywheelFront.set(flywheelsTargetVelocity / 2800);
        flywheelBack.set(flywheelFront.get());
        flywheelsRunning = true;
    }

    public void setPower(double power) {
        flywheelFront.set(power);
        flywheelBack.set(flywheelFront.get());
    }

    public void stop() {
        flywheelFront.set(0);
        flywheelBack.set(0);
        flywheelsRunning = false;
    }

    public boolean flywheelsRunning() {
        return flywheelsRunning;
    }

    public double getTargetVelocity() {
        return flywheelsTargetVelocity;
    }

    public double getActualVelocity() {
        return flywheelFront.get();
    }

    public void setTargetVelocity(double velocity) {
        flywheelFront.set(velocity);
        flywheelBack.set(flywheelFront.get());
    }

    public void toggle() {
        if (flywheelsRunning)
            stop();
        else start();
    }
}
