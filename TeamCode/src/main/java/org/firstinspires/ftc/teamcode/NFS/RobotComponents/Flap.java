package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;

/**
 * @author Topik
 * @version 1.1
 * @since 1.0
 * This class controls the movement of the aiming flap
 */
@Config
public class Flap {
    /**
     * The various positions of the flap
     */

    //highgoalpos = .28
    public static double startPosition = 0, highGoalPosition = .276, autoHighGoalPosition = .275, powerShotPosition = .3, slowPowerShotPosition = .275, flushPosition = .1, adjustUpThreshold = .005, adjustDownThreshold = .005;
    /**
     * The flap Servo object
     */
    public Servo flap;
    private final Robot robot;
    private State state;

    private enum State {
        HIGH_GOAL,
        POWERSHOT,
        FLUSH,
        NEUTRAL,
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
            throw new BadInitializationException("Null robot in flap detected");
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
     * Sets the flap to the auto high goal position
     */
    public void goToAutoHighGoalPosition() {
        flap.setPosition(autoHighGoalPosition);
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

    public void goToSlowPowershotPosition(){
        flap.setPosition(slowPowerShotPosition);
        state = State.POWERSHOT;
        robot.flywheels.setSlowPowerShotVelocity();
    }

    public void goFlush() {
        flap.setPosition(flushPosition);
        state = State.FLUSH;
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

    /**
     * Checks if the flap is vertically flush with the rest of the robot
     * @return true if the flap is flush
     */
    public boolean isFlush() {
        return state == State.FLUSH;
    }

    /**
     * Gets the current state of the flap
     *
     * @return the state of the flap as a State
     */
    public State getState() {
        return state;
    }
}
