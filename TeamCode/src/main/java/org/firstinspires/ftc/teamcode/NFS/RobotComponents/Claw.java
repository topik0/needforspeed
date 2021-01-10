package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    public Servo claw;
    private State state;
    public static double closedPosition = .42, openPosition = .74;

    private enum State {
        OPEN,
        CLOSED
    }

    public Claw(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Claw genesis is null");
        claw = gen.claw;
        if (claw == null)
            throw new BadInitializationException("Claw is null");
        setStartPosition();
    }

    public void setStartPosition() {
        claw.setPosition(closedPosition);
    }

    public void open() {
        claw.setPosition(openPosition);
        state = State.OPEN;
    }

    public void close() {
        claw.setPosition(closedPosition);
        state = State.CLOSED;
    }

    public boolean isClosed() {
        return state == State.CLOSED;
    }

    public void toggle() {
        if (isClosed()) open();
        else close();
    }
}
