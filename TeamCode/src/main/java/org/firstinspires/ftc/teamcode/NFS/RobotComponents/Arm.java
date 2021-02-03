package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class controls the wobble arm mechanism
 */
@Config
public class Arm {
    /**
     * The arm servo object
     */
    public Servo arm;
    /**
     * The various servo positions
     */
    public static double startPosition = .73, upPosition = .73, downPosition = .115;
    private State state;

    private enum State {
        UP,
        DOWN
    }

    /**
     * Arm Constructor
     *
     * @param gen the HardwareGenesis object needed to grab the arm servo
     */
    public Arm(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Null Genesis detected in Arm");
        arm = gen.arm;
        if (arm == null)
            throw new BadInitializationException("Null arm detected in Arm");
    }

    /**
     * Sets the arm to the start position
     */
    public void setStartPosition() {
        arm.setPosition(startPosition);
        state = State.UP;
    }

    /**
     * Moves the arm to the up position
     */
    public void up() {
        arm.setPosition(upPosition);
        state = State.UP;
    }

    /**
     * Moves the arm to the down position
     */
    public void down() {
        arm.setPosition(downPosition);
        state = State.DOWN;
    }

    /**
     * Checks if the arm is up
     *
     * @return true if the arm is up
     */
    public boolean isUp() {
        return state == State.UP;
    }

    /**
     * Gets the state of the arm
     *
     * @return the arm state
     */
    public State getState() {
        return state;
    }

    /**
     * Sets the position of the arm
     *
     * @param position the position to be set
     */
    public void setPosition(double position) {
        arm.setPosition(position);
    }

    /**
     * Toggles the arm state between up and down
     */
    public void toggle() {
        if (isUp()) down();
        else up();
    }
}
