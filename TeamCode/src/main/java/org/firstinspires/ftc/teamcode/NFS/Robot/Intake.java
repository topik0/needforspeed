package org.firstinspires.ftc.teamcode.NFS.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public DcMotor leftIntake, rightIntake;
    public Servo stopper;
    private State state;
    private State verticalState;
    private State directionState;
    public static double runPower = 1.0, stopPower = 0;
    public static double downPosition = 1.0, stopperStartPosition = .85;
    private double currentRunPower = 0;

    private enum State {
        RUNNING,
        DORMANT,
        DOWN,
        UP,
        REVERSED,
        FORWARD
    }

    public Intake(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Intake genesis is null");
        leftIntake = gen.leftIntake;
        rightIntake = gen.rightIntake;
        stopper = gen.intakeStopper;
        if ((leftIntake == null || rightIntake == null) || stopper == null)
            throw new BadInitializationException("Intake or stopper is null");
        setInitialState();
    }

    private void setInitialState() {
        state = State.DORMANT;
        verticalState = State.UP;
        directionState = State.FORWARD;
    }

    public void stopperStartPosition() {
        stopper.setPosition(stopperStartPosition);
    }

    public void setPower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
        currentRunPower = power;
        if (Math.abs(power) >= .00001)
            state = State.RUNNING;
        else state = State.DORMANT;
        if (power > 0) directionState = State.FORWARD;
        else if (power < 0) directionState = State.REVERSED;
    }

    public double getPower() {
        return currentRunPower;
    }

    public void start() {
        setPower(runPower);
    }

    public void stop() {
        setPower(stopPower);
    }

    public void toggle() {
        if (isRunning()) stop();
        else start();
    }

    public void reverse() {
        setPower(-runPower);
    }

    public void down() {
        stopper.setPosition(downPosition);
        verticalState = State.DOWN;
    }

    public State getState() {
        return state;
    }

    public State getDirectionState() {
        return directionState;
    }

    public State getVerticalState() {
        return verticalState;
    }

    public boolean isRunning() {
        return state == State.RUNNING;
    }

    public boolean isUp() {
        return verticalState == State.UP;
    }

    public boolean isReversed() {
        return directionState == State.REVERSED;
    }
}
