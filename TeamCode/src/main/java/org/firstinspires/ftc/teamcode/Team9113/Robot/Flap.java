package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * <p>
 * This class controls the movement of the aiming flap
 */
@Config
public class Flap {
    private final Robot robot;
    public Servo flap;
    private State state;
    public static double highGoalPosition = .2785, powerShotPosition = .3, adjustUpThreshold = .005, adjustDownThreshold = .005;

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
            flap.setPosition(powerShotPosition);
            state = State.POWERSHOT;
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
     * Gets the current
     *
     * @return
     */
    public State getState() {
        return state;
    }

    public void setPosition(double position) {
        flap.setPosition(position);
    }

    public boolean isHighGoal() {
        return state == State.HIGH_GOAL;
    }

    public boolean isNeutral() {
        return state == State.NEUTRAL;
    }
}
