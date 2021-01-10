package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

@Config
public class Flicker {
    private Robot robot;
    public static double startPosition = .55, outPosition = .65, inPosition = .55;
    public static double cooldown = 60;
    public static boolean noShootWhileFlywheelsDormant = true;
    private SampleMecanumDrive drive;
    public Servo flicker;
    private State state = State.IN;
    private double startTime = 0;

    private enum State {
        OUT,
        IN
    }

    public Flicker(HardwareGenesis gen, SampleMecanumDrive drive, Robot robot) {
        this.robot = robot;
        this.drive = drive;
        flicker = gen.flicker;
    }

    public void startPosition() {
        flicker.setPosition(startPosition);
    }

    public void shootOut() {
        if ((noShootWhileFlywheelsDormant && !robot.flywheels.running()) || state == State.OUT)
            return;
        flicker.setPosition(outPosition);
        state = State.OUT;
        startTime = System.currentTimeMillis();
    }

    public void bringIn() {
        if (state == State.IN) return;
        flicker.setPosition(inPosition);
        state = State.IN;
        startTime = 0;
    }

    public void checkState() {
        if (state == State.OUT && reachedCooldown())
            bringIn();
    }

    private boolean reachedCooldown() {
        return System.currentTimeMillis() - startTime >= cooldown;
    }

    public State flickerState() {
        return state;
    }

    /*
    FOR USE IN AUTONOMOUS PROGRAMS ONLY
     */
    public void launch() {
        shootOut();
        startTime = System.currentTimeMillis();
        while (!reachedCooldown())
            drive.update();
        startTime = 0;
        bringIn();
    }
}
