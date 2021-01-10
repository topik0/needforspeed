package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    public Servo arm;
    public static double startPosition = .69, upPosition = .69, downPosition = .07;
    private State state;

    private enum State {
        UP,
        DOWN
    }

    public Arm(HardwareGenesis gen) {
        if (gen == null)
            throw new BadInitializationException("Arm genesis is null");
        arm = gen.arm;
        if (arm == null)
            throw new BadInitializationException("Arm is null");
    }

    public void setStartPosition() {
        arm.setPosition(startPosition);
        state = State.UP;
    }

    public void up() {
        arm.setPosition(upPosition);
        state = State.UP;
    }

    public void down() {
        arm.setPosition(downPosition);
        state = State.DOWN;
    }

    public boolean isUp() {
        return state == State.UP;
    }

    public void setPosition(double position) {
        arm.setPosition(position);
    }

    public void toggle() {
        if (isUp()) down();
        else up();
    }
}
