package org.firstinspires.ftc.teamcode.Team9113;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    /**
     * upperLeft [0] upperRight [1]
     * lowerLeft [2] lowerRight [3]
     */

    public double normalDrive = 1;
    public double snailDrive = .5;
    public double currentThrottle = normalDrive;

    public Motor[] drivetrain = new Motor[4];
    protected HardwareMap hwMap;

    public Drivetrain(HardwareMap hwMap) {
        this.hwMap = hwMap;
        for (int i = 0; i < 4; i++) {
            String[] drivetrainNames = {"upperLeft", "upperRight", "lowerLeft", "lowerRight"};
            drivetrain[i] = new Motor(this.hwMap, drivetrainNames[i], Motor.GoBILDA.RPM_312);
            drivetrain[i].setRunMode(Motor.RunMode.RawPower);
        }
    }

    public void moveLeft() {
        drivetrain[0].set(-currentThrottle);
        drivetrain[1].set(currentThrottle);
        drivetrain[2].set(currentThrottle);
        drivetrain[3].set(-currentThrottle);
    }

    public void moveRight() {
        drivetrain[0].set(-currentThrottle);
        drivetrain[1].set(currentThrottle);
        drivetrain[2].set(currentThrottle);
        drivetrain[3].set(-currentThrottle);
    }

    public void driveForward() {
        for (int i = 0; i < 4; i++) {
            drivetrain[i].set(-currentThrottle);
        }
    }

    public void driveBackward() {
        for (int i = 0; i < 4; i++) {
            drivetrain[i].set(currentThrottle);
        }
    }

    public void toggleNormalDrive() {
        if (!isNormalDriveEnabled()) {
            enableNormalDrive();
            return;
        }
        disableNormalDrive();
    }

    public void enableNormalDrive() {
        currentThrottle = normalDrive;
    }

    public void disableNormalDrive() {
        currentThrottle = snailDrive;
    }

    public void toggleSnailDrive() {
        if (!isSnailDriveEnabled()) {
            enableSnailDrive();
        }
    }

    public void enableSnailDrive() {
        currentThrottle = snailDrive;
    }

    public void disableSnailDrive() {
        currentThrottle = normalDrive;
    }

    public boolean isSnailDriveEnabled() {
        return currentThrottle == snailDrive;
    }

    public boolean isNormalDriveEnabled() {
        return currentThrottle == normalDrive;
    }

    public void brakeDrivetrain() {
        for (Motor motor : drivetrain)
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setFieldCentricPower(double y, double x, double rx) {
        drivetrain[2].set(y - x + rx);
        drivetrain[0].set(y + x + rx);
        drivetrain[1].set(y - x - rx);
        drivetrain[3].set(y + x - rx);
    }

    public void setPower(double power) {
        for (int i = 0; i < 4; i++) {
            drivetrain[i].set(power);
        }
    }

    public void setLeftPower(double power) {
        for (int i = 0; i <= 2; i += 2) {
            drivetrain[i].set(power);
        }
    }

    public void setRightPower(double power) {
        for (int i = 1; i <= 3; i += 2) {
            drivetrain[i].set(power);
        }
    }

    public double getThrottle() {
        return currentThrottle;
    }
}
