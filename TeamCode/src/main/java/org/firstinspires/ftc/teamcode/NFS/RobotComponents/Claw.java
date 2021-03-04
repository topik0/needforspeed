package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class controls the claw mechanism
 */
@Config
public class Claw {
    /**
     * The claw servo object
     */
    public Servo claw;
    /**
     * The open and close positions for the claw
     */
    public static double closedPosition = .23, openPosition = .6;
    private State state;

    private enum State {
        OPEN,
        CLOSED
    }

    /**
     * Claw Constructor
     *
     * @param gen the HardwareGenesis object needed to get the claw servo
     */
    public Claw(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Claw genesis is null");
        claw = gen.claw;
        if (claw == null)
            throw new BadInitializationException("Claw is null");
        setStartPosition();
        state = State.CLOSED;
    }

    /**
     * Sets the claw to the start position
     */
    public void setStartPosition() {
        claw.setPosition(closedPosition);
    }

    /**
     * Opens the claw
     */
    public void open() {
        claw.setPosition(openPosition);
        state = State.OPEN;
    }

    /**
     * Closes the claw
     */
    public void close() {
        claw.setPosition(closedPosition);
        state = State.CLOSED;
    }

    /**
     * Checks if the claw is closed
     *
     * @return true if the claw is closed
     */
    public boolean isClosed() {
        return state == State.CLOSED;
    }

    /**
     * Toggles the claw between open and closed
     */
    public void toggle() {
        if (isClosed()) open();
        else close();
    }

    /**
     * Gets the claw state
     *
     * @return returns the claw state
     */
    public State getState() {
        return state;
    }
}
