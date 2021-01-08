package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Flywheels {
    public Motor flywheelFront, flywheelBack;
    private State runState;
    private State velocityState;
    public static double targetVelocity = 0;
    public static double maxVelocity = 1900;
    public static double power = 0;
    public static double kP = 12, kI = 12, kD = 0.3;

    private enum State {
        RUNNING,
        DORMANT,
        LOWER_VELOCITY,
        MAX_VELOCITY
    }

    public Flywheels(HardwareGenesis gen) {
        flywheelFront = gen.flywheelFront;
        flywheelFront.setRunMode(Motor.RunMode.VelocityControl);
        flywheelFront.setVeloCoefficients(kP, kI, kD);
        flywheelBack = gen.flywheelBack;
        runState = State.DORMANT;
        setVelocityState();
        setRunningState();
    }

    public void run() {
        setPower(targetVelocity / 2800);
        setVelocityState();
        setRunningState();
    }

    public void halt() {
        targetVelocity = 0;
        setVelocityState();
        setRunningState();
        setPower(0);
    }

    public void setRunningState() {
        if (targetVelocity <= .001) runState = State.DORMANT;
        else runState = State.RUNNING;
    }

    public void setVelocityState() {
        if (targetVelocity <= .001)
            velocityState = State.DORMANT;
        else if (doubleEquals(targetVelocity, maxVelocity))
            velocityState = State.MAX_VELOCITY;
        else
            velocityState = State.LOWER_VELOCITY;
    }

    public void start() {
        setPower(maxVelocity / 2800);
        runState = State.RUNNING;
    }

    public void setPower(double power) {
        Flywheels.power = power;
        flywheelFront.set(power);
        flywheelBack.set(flywheelFront.get());
        if (Math.abs(power) <= .001) runState = State.DORMANT;
        else runState = State.RUNNING;
    }

    public void stop() {
        flywheelFront.set(0);
        flywheelBack.set(0);
        runState = State.RUNNING;
    }

    public State getRunState() {
        return runState;
    }

    public State getVelocityState() {
        return velocityState;
    }

    public boolean atMaxVelocity() {
        return velocityState == State.MAX_VELOCITY;
    }

    public boolean running() {
        return runState == State.RUNNING;
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
        if (running())
            stop();
        else start();
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
