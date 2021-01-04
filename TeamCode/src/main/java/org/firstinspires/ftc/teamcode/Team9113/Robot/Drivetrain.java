package org.firstinspires.ftc.teamcode.Team9113.Robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Team9113.drive.SampleMecanumDrive;

public class Drivetrain {
    /**
     * upperLeft [0] upperRight [1]
     * lowerLeft [2] lowerRight [3]
     */

    public double normalDrive = 1;
    public double snailDrive = .65;
    public double turnThrottle = .75;
    public double snailTurnThrottle = .4;
    public double currentThrottle = normalDrive;
    public double currentTurnThrottle = turnThrottle;

    public Motor[] motors = new Motor[4];
    protected HardwareMap hwMap;
    public SampleMecanumDrive mecanumDrive;

    public Drivetrain(HardwareMap hwMap) {
        this.hwMap = hwMap;
        for (int i = 0; i < 4; i++) {
            String[] drivetrainNames = {"lowerRight", "lowerLeft", "upperRight", "upperLeft"};
            motors[i] = new Motor(this.hwMap, drivetrainNames[i], Motor.GoBILDA.RPM_435);
            motors[i].setRunMode(Motor.RunMode.RawPower);
        }
        mecanumDrive = new SampleMecanumDrive(hwMap);
        reverseAll();
    }

    public void driveFieldCentric(double x, double y, double rx) {
        motors[0].set(y - x + rx);
        motors[1].set(y + x + rx);
        motors[2].set(y - x - rx);
        motors[3].set(y + x - rx);
    }

    public void toggleNormalDrive() {
        if (!isNormalDriveEnabled())
            enableNormalDrive();
        else
            disableNormalDrive();
    }

    public void enableNormalDrive() {
        currentThrottle = normalDrive;
        currentTurnThrottle = turnThrottle;
    }

    public void disableNormalDrive() {
        currentThrottle = snailDrive;
        currentTurnThrottle = snailTurnThrottle;
    }

    public void toggleSnailDrive() {
        if (!isSnailDriveEnabled())
            enableSnailDrive();
        else
            disableSnailDrive();
    }

    public void enableSnailDrive() {
        currentThrottle = snailDrive;
        currentTurnThrottle = snailTurnThrottle;
    }

    public void disableSnailDrive() {
        currentThrottle = normalDrive;
        currentTurnThrottle = turnThrottle;
    }

    public boolean isSnailDriveEnabled() {
        return currentThrottle == snailDrive;
    }

    public boolean isNormalDriveEnabled() {
        return currentThrottle == normalDrive;
    }

    public void brake() {
        for (Motor motor : motors)
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void reverseAll() {
        for (Motor motor : motors)
            motor.setInverted(true);
    }

    public void setPower(double power) {
        for (Motor motor : motors)
            motor.set(power);
    }

    public void setLeftPower(double power) {
        motors[0].set(power);
        motors[2].set(power);
    }

    public void setRightPower(double power) {
        motors[1].set(power);
        motors[3].set(power);
    }

    public double getThrottle() {
        return currentThrottle;
    }
}