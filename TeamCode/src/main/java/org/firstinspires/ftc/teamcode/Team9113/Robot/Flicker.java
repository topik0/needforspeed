package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.lang3.time.StopWatch;
import org.firstinspires.ftc.teamcode.Team9113.drive.SampleMecanumDrive;

@Config
public class Flicker {
    private HardwareMap hwMap;
    public static String flickerName = "flicker";
    public static double startPosition = .55, outPosition = .65, inPosition = .55;
    public static double cooldown = 80;
    private SampleMecanumDrive drive;
    public Servo flicker;
    private State state = State.IN;
    private StopWatch stopwatch;

    private enum State {
        OUT,
        IN
    }

    public Flicker(HardwareMap hwMap, SampleMecanumDrive drive) {
        this.hwMap = hwMap;
        this.drive = drive;
        stopwatch = new StopWatch();
        initServo();
    }

    private void initServo() {
        flicker = hwMap.servo.get(flickerName);
    }

    public void startPosition() {
        flicker.setPosition(startPosition);
    }

    public void shootOut() {
        flicker.setPosition(outPosition);
        state = State.OUT;
        stopwatch.start();
    }

    public void bringIn() {
        if (state == State.IN) return;
        flicker.setPosition(inPosition);
        state = State.IN;
        stopwatch.reset();
    }

    public void checkState(){
        if (state == State.OUT || timeNotAtCooldown())
            bringIn();
    }

    private boolean timeNotAtCooldown(){
        return stopwatch.getTime() <= cooldown;
    }

    /*
    FOR USE IN AUTONOMOUS PROGRAMS ONLY
     */
    public void launch(){
        shootOut();
        stopwatch.start();
        while(timeNotAtCooldown())
            drive.update();
        stopwatch.reset();
        bringIn();
    }
}
