package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class controls the flicker which shoots the rings
 */
@Config
public class Flicker {
    /**
     * The various positions of the flicker
     */
    public static double startPosition = .56, outPosition = .67, inPosition = .56; //out pose used to be .65
    /**
     * The delay between movements
     */
    public static double cooldown = 70;
    /**
     * Controls if the flicker should shoot if the flywheels aren't running
     */
    public static boolean noShootWhileFlywheelsDormant = true;
    /**
     * The flicker servo
     */
    public Servo flicker;
    private final Robot robot;
    private final SampleMecanumDrive drive;
    private State state = State.IN;
    private double startTime = 0;

    private enum State {
        OUT,
        IN
    }

    /**
     * Flicker constructor
     *
     * @param gen   the HardwareGenesis object needed to get the flicker servo
     * @param robot the robot object needed to use robot functions
     */
    public Flicker(HardwareGenesis gen, Robot robot) {
        if (gen == null)
            throw new BadInitializationException("Null Genesis detected in flicker");
        flicker = gen.flicker;
        if (flicker == null)
            throw new BadInitializationException("Null Flicker Servo detected in flicker");
        this.robot = robot;
        if (robot == null)
            throw new BadInitializationException("Null Robot detected in flicker");
        this.drive = robot.drivetrain.mecanumDrive;
    }

    /**
     * Sets the flicker to the start position
     */
    public void startPosition() {
        flicker.setPosition(startPosition);
    }

    /**
     * Moves the flicker to the out position
     */
    public void shootOut() {
        if (state == State.OUT || (noShootWhileFlywheelsDormant && !robot.flywheels.running()))
            return;
        flicker.setPosition(outPosition);
        state = State.OUT;
        startTime = System.currentTimeMillis();
    }

    /**
     * Brings the flicker to the in position
     */
    public void bringIn() {
        if (state == State.IN) return;
        flicker.setPosition(inPosition);
        state = State.IN;
        startTime = 0;
    }

    /**
     * Checks the current state of the flicker, and decides whether it needs to be brought in
     */
    public void checkState() {
        if (state == State.OUT && reachedCooldown())
            bringIn();
    }

    /**
     * Checks if the cooldown in between movements has been reached
     *
     * @return true if the cooldown between movements has been reached
     */
    private boolean reachedCooldown() {
        return System.currentTimeMillis() - startTime >= cooldown;
    }

    /**
     * Gets the flicker state
     *
     * @return the flicker state
     */
    public State getState() {
        return state;
    }

    /**
     * Shoots a ring and loops PID for the cooldown (recommended use only in autonomous programs)
     */
    public void launch() {
        shootOut();
        robot.delayWithAllPID(cooldown);
        bringIn();
    }
}
