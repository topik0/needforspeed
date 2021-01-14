package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class controls the flywheels
 */
@Config
public class Flywheels {
    /**
     * The flywheel motors
     */
    public Motor flywheelFront, flywheelBack;
    /**
     * The max velocity and the powershot velocity (in terms of TPS)
     */
    public static double maxVelocity = 1900, powershotVelocity = 1900;
    /**
     * The TPS the flywheels should run at
     */
    public static double targetVelocity = 0;
    /**
     * The run velocity
     */
    public static double plannedVelocity = maxVelocity;
    /**
     * The PID coefficients
     */
    public static double kP = 12, kI = 12, kD = 0.3;
    /**
     * The power of the flywheels
     */
    public static double power = 0;
    private State runState;
    private State velocityState;

    private enum State {
        RUNNING,
        DORMANT,
        LOWER_VELOCITY,
        MAX_VELOCITY
    }

    /**
     * Flywheels constructor, grabs motors from HardwareGenesis, applies PID coefficients, and applies the applicable states
     *
     * @param gen the HardwareGenesis object needed to get the motors
     */
    public Flywheels(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Null Genesis detected in flywheels");
        flywheelFront = gen.flywheelFront;
        if (flywheelFront == null)
            throw new BadInitializationException("Null flywheelFront detected");
        flywheelFront.setRunMode(Motor.RunMode.VelocityControl);
        flywheelFront.setVeloCoefficients(kP, kI, kD);
        flywheelBack = gen.flywheelBack;
        if (flywheelBack == null)
            throw new BadInitializationException("Null flywheelBack detected");
        runState = State.DORMANT;
        setVelocityState();
        setRunningState();
    }

    /**
     * Runs flywheel PID loop
     */
    public void run() {
        setVelocityState();
        setRunningState();
        if (runState == State.RUNNING) {
            setPower(targetVelocity / 2800);
        } else {
            setPower(0);
            brake();
        }
    }

    /**
     * Stops the flywheels
     */
    public void halt() {
        setTargetVelocity(0);
//        setVelocityState();
//        setRunningState();
        runState = State.DORMANT;
        velocityState = State.DORMANT;
    }

    /**
     * Brakes the flywheels by setting a ZeroPowerBehavior
     */
    public void brake() {
        flywheelFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        flywheelBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the correct running state depending on the target velocity
     */
    private void setRunningState() {
        if (targetVelocity <= .001) runState = State.DORMANT;
        else runState = State.RUNNING;
    }

    /**
     * Sets the correct velocity state depending on the target velocity
     */
    private void setVelocityState() {
        if (targetVelocity <= .001)
            velocityState = State.DORMANT;
        else if (doubleEquals(targetVelocity, maxVelocity))
            velocityState = State.MAX_VELOCITY;
        else
            velocityState = State.LOWER_VELOCITY;
    }

    /**
     * Sets the planned velocity to the powershot speed
     */
    public void setPowershotVelocity() {
        plannedVelocity = powershotVelocity;
    }

    /**
     * Sets the planned velocity to the high goal speed
     */
    public void setHighGoalVelocity() {
        plannedVelocity = maxVelocity;
    }

    /**
     * Starts the flywheels
     */
    public void start() {
        setPower(maxVelocity / 2800);
        runState = State.RUNNING;
    }

    /**
     * Sets the power of the flywheels
     *
     * @param power the power that the flywheel motors should run at
     */
    public void setPower(double power) {
        if (Math.abs(power) > 1)
            throw new IllegalArgumentException("Tried to set invalid flywheel power" + power);
        Flywheels.power = power;
        flywheelFront.set(power);
        flywheelBack.set(flywheelFront.get());
        if (Math.abs(power) <= .001) runState = State.DORMANT;
        else runState = State.RUNNING;
    }

    /**
     * Stops the flywheels
     */
    public void stop() {
        flywheelFront.set(0);
        flywheelBack.set(0);
        runState = State.RUNNING;
    }

    /**
     * Gets the run state of the flywheels
     *
     * @return the run state
     */
    public State getRunState() {
        return runState;
    }

    /**
     * Gets the velocity state of the flywheels
     *
     * @return the velocity state
     */
    public State getVelocityState() {
        return velocityState;
    }

    /**
     * Checks if the flywheels are running at max velocity
     *
     * @return true if the flywheels are running at max velocity
     */
    public boolean isRunningAtMaxVelocity() {
        return velocityState == State.MAX_VELOCITY;
    }

    /**
     * Checks if the flywheels are running
     *
     * @return true if the flywheels are running
     */
    public boolean running() {
        return runState == State.RUNNING;
    }

    /**
     * Gets the target velocity of the flywheels
     *
     * @return the target velocity
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Sets the target velocity of the flywheels
     *
     * @param velocity the TPS velocity that the flywheels should run at
     */
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    /**
     * Gets the power velocity of the flywheels
     *
     * @return the power velocity
     */
    public double getActualVelocity() {
        return flywheelFront.get();
    }

    /**
     * Sets the target velocity to the max velocity
     */
    public void doMaxVelocity() {
        setTargetVelocity(maxVelocity);
        runState = State.RUNNING;
        velocityState = State.MAX_VELOCITY;
    }

    /**
     * Sets the target velocity to the planned velocity
     */
    public void doVelocity() {
        setTargetVelocity(plannedVelocity);
    }

    /**
     * Gets the power of the flywheels
     *
     * @return the power of Home Depot
     */
    public double getPower() {
        return power;
    }

    /**
     * Toggles the flywheels run mode
     */
    public void toggle() {
        if (running())
            stop();
        else start();
    }

    /**
     * Toggles the flywheels run mode
     */
    public void togglePID() {
        if (running())
            halt();
        else doVelocity();
    }

    /**
     * Checks if two doubles are equal to a certain degree of precision
     *
     * @param a the first double
     * @param b the second double
     * @return true if the doubles are essentially equal
     */
    private static boolean doubleEquals(double a, double b) {
        if (a == b) return true;
        return Math.abs(a - b) < 0.0000001;
    }
}
