package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.lang3.time.StopWatch;
import org.firstinspires.ftc.teamcode.Team9113.drive.SampleMecanumDrive;

@Config
public class Flicker {
    private Robot robot;
    public static double startPosition = .55, outPosition = .65, inPosition = .55;
    public static double cooldown = 80;
    public static boolean noShootWhileFlywheelsDormant = true;
    private SampleMecanumDrive drive;
    public Servo flicker;
    private State state = State.IN;
    private StopWatch stopwatch;

    private enum State {
        OUT,
        IN
    }

    public Flicker(HardwareGenesis gen, SampleMecanumDrive drive, Robot robot) {
        this.robot = robot;
        this.drive = drive;
        flicker = gen.flicker;
        stopwatch = new StopWatch();
    }

    public void startPosition() {
        flicker.setPosition(startPosition);
    }

    public void shootOut() {
        if (noShootWhileFlywheelsDormant && !robot.flywheels.running()) return;
        flicker.setPosition(outPosition);
        state = State.OUT;
        stopwatch.stop();
        stopwatch.reset();
    }

    public void bringIn() {
        if (state == State.IN) return;
        flicker.setPosition(inPosition);
        state = State.IN;
        stopwatch.stop();
        stopwatch.reset();
    }

    public void checkState() {
        if (state == State.OUT && !timeNotAtCooldown())
            bringIn();
    }

    private boolean timeNotAtCooldown() {
        return stopwatch.getTime() <= cooldown;
    }

    /*
    FOR USE IN AUTONOMOUS PROGRAMS ONLY
     */
    public void launch() {
        shootOut();
        stopwatch.start();
        while (timeNotAtCooldown())
            drive.update();
        stopwatch.stop();
        stopwatch.reset();
        bringIn();
    }
}
