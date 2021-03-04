package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class controls the intake and intake stopper
 */
@Config
public class Intake {
    /**
     * The intake motors
     */
    public DcMotor leftIntake, rightIntake;
    /**
     * The intake stopper servo
     */
    public Servo stopper;
    /**
     * The power that the intake is run at when it is running and stopped
     */
    public static double runPower = 1.0, reverseRunPower = -1.0, stopPower = 0;
    /**
     * The positions for the intake stopper
     */
    public static double downPosition = .85, stopperStartPosition = 1;
    private State state;
    private State verticalState;
    private State directionState;
    private double currentRunPower = 0;

    private enum State {
        RUNNING,
        DORMANT,
        DOWN,
        UP,
        REVERSED,
        FORWARD
    }

    /**
     * Intake constructor
     *
     * @param gen the hardware genesis object
     */
    public Intake(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Null Genesis detected in intake");
        leftIntake = gen.leftIntake;
        rightIntake = gen.rightIntake;
        stopper = gen.intakeStopper;
        if (leftIntake == null || rightIntake == null)
            throw new BadInitializationException("Null intake motor(s) detected in intake");
        if (stopper == null)
            throw new BadInitializationException("Null Stopper Servo detected in intake");
        setInitialState();
    }

    /**
     * Sets the initial states of the intake stopper and intake
     */
    private void setInitialState() {
        state = State.DORMANT;
        verticalState = State.UP;
        directionState = State.FORWARD;
    }

    /**
     * Brings the intake stopper to the start position
     */
    public void stopperStartPosition() {
        stopper.setPosition(stopperStartPosition);
    }

    /**
     * Sets the power of the intake and sets the states accordingly
     *
     * @param power the power that the intake motors will run at
     */
    public void setPower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
        currentRunPower = power;
        if (Math.abs(power) >= .001)
            state = State.RUNNING;
        else state = State.DORMANT;
        if (power > 0) directionState = State.FORWARD;
        else if (power < 0) directionState = State.REVERSED;
    }

    /**
     * Gets the current run power of the intake motors
     *
     * @return the run power of the intake motors
     */
    public double getPower() {
        return currentRunPower;
    }

    /**
     * Starts the intake
     */
    public void start() {
        setPower(runPower);
        state = State.RUNNING;
    }

    /**
     * Stops the intake
     */
    public void stop() {
        setPower(stopPower);
        state = State.DORMANT;
    }

    /**
     * Toggles the intake state from running to dormant, or dormant to running
     */
    public void toggle() {
        if (isRunning()) stop();
        else start();
    }

    /**
     * Reverses the intake direction
     */
    public void reverse() {

        setPower(reverseRunPower);
        state = State.RUNNING;
    }

    /**
     * Moves the intake down
     */
    public void down() {
        stopper.setPosition(downPosition);
        verticalState = State.DOWN;
    }

    /**
     * Gets the current state of the intake
     *
     * @return the intake state
     */
    public State getState() {
        return state;
    }

    /**
     * Gets the direction state of the intake
     *
     * @return the intake direction state
     */
    public State getDirectionState() {
        return directionState;
    }

    /**
     * Gets the vertical state of the intake (whether or not the intake is down)
     *
     * @return the vertical state of the intake
     */
    public State getVerticalState() {
        return verticalState;
    }

    /**
     * Checks if the intake is running
     *
     * @return true if the intake is running
     */
    public boolean isRunning() {
        return state == State.RUNNING;
    }

    /**
     * Checks if the intake is up
     *
     * @return true if the intake is up
     */
    public boolean isUp() {
        return verticalState == State.UP;
    }

    /**
     * Checks if the intake is reversed
     *
     * @return true if the intake is reversed
     */
    public boolean isReversed() {
        return directionState == State.REVERSED;
    }
}
