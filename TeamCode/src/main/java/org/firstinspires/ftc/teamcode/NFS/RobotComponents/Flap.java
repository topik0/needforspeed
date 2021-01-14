package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class controls the movement of the aiming flap
 */
@Config
public class Flap {
    /**
     * The various positions of the flap
     */
    public static double startPosition = 0, highGoalPosition = .2785, powerShotPosition = .3, adjustUpThreshold = .005, adjustDownThreshold = .005;
    /**
     * The flap Servo object
     */
    public Servo flap;
    private final Robot robot;
    private State state;

    private enum State {
        HIGH_GOAL,
        POWERSHOT,
        NEUTRAL
    }

    /**
     * Flap constructor that grabs the flap
     *
     * @param gen   the HardwareGenesis object needed to grab the flap
     * @param robot the robot that contains the robot objects
     */
    public Flap(HardwareGenesis gen, Robot robot) {
        if (gen == null)
            throw new BadInitializationException("Flap genesis is null");
        if (robot == null)
            throw new BadInitializationException("Robot given to flap is null");
        this.robot = robot;
        flap = gen.flap;
        if (flap == null)
            throw new BadInitializationException("Flap is null");
        setStartPosition();
    }

    /**
     * Sets the flap to its stop position
     */
    public void setStartPosition() {
        if (robot.isAuto()) {
            flap.setPosition(startPosition);
            state = State.NEUTRAL;
        } else {
            flap.setPosition(highGoalPosition);
            state = State.HIGH_GOAL;
        }
    }

    /**
     * Sets the flap to the high goal position
     */
    public void goToHighGoalPosition() {
        flap.setPosition(highGoalPosition);
        state = State.HIGH_GOAL;
        robot.flywheels.setHighGoalVelocity();
    }

    /**
     * Sets the flap to the powershot position
     */
    public void goToPowershotPosition() {
        flap.setPosition(powerShotPosition);
        state = State.POWERSHOT;
        robot.flywheels.setPowershotVelocity();
    }

    /**
     * Raises the flap up an increment
     */
    public void adjustUp() {
        flap.setPosition(flap.getPosition() - adjustUpThreshold);
    }

    /**
     * Lowers the flap an increment
     */
    public void adjustDown() {
        flap.setPosition(flap.getPosition() + adjustDownThreshold);
    }

    /**
     * Gets the current state of the flap
     *
     * @return the state of the flap as a State
     */
    public State getState() {
        return state;
    }

    /**
     * Sets the flap to a specified position
     *
     * @param position the position to set the flap to
     */
    public void setPosition(double position) {
        if (Math.abs(position) > 1 || position < 0)
            throw new IllegalArgumentException("Tried to set invalid flap position: " + position);
        flap.setPosition(position);
    }

    /**
     * Checks if the flap is in the high goal position
     *
     * @return true if the flap is in the high goal position
     */
    public boolean isInHighGoalPosition() {
        return state == State.HIGH_GOAL;
    }

    /**
     * Checks if the flap is in a neutral state (not in powershot or high goal position)
     *
     * @return true if the flap is neutral
     */
    public boolean isNeutral() {
        return state == State.NEUTRAL;
    }
}
