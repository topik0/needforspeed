package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Flap {
    public Servo flap;
    private State state;
    public static double highGoalPosition = .2785, powerShotPosition = .3, adjustUpThreshold = .005, adjustDownThreshold = .005;

    private enum State {
        HIGH_GOAL,
        POWERSHOT,
        NEUTRAL
    }

    public Flap(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Flap genesis is null");
        flap = gen.flap;
        if (flap == null)
            throw new BadInitializationException("Flap is null");
        setStartPosition();
    }

    public void setStartPosition() {
        flap.setPosition(powerShotPosition);
        state = State.POWERSHOT;
    }

    public void goToHighGoalPosition() {
        flap.setPosition(highGoalPosition);
        state = State.HIGH_GOAL;
    }

    public void goToPowershotPosition() {
        flap.setPosition(powerShotPosition);
        state = State.POWERSHOT;
    }

    public void adjustUp() {
        flap.setPosition(flap.getPosition() - adjustUpThreshold);
    }

    public void adjustDown() {
        flap.setPosition(flap.getPosition() + adjustDownThreshold);
    }

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
