package org.firstinspires.ftc.teamcode.NFS.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

@Config
public class Drivetrain {
    /**
     * upperLeft [0] upperRight [1]
     * lowerLeft [2] lowerRight [3]
     */

    public static double normalThrottle = 1;
    public static double snailDrive = .65;
    public static double turnThrottle = .75;
    public static double snailTurnThrottle = .4;
    public static double currentThrottle = normalThrottle;
    public static double currentTurnThrottle = turnThrottle;

    public DcMotorEx[] motors;
    public SampleMecanumDrive mecanumDrive;

    public Drivetrain(HardwareGenesis gen) {
        motors = gen.drivetrainMotors;
        mecanumDrive = new SampleMecanumDrive(gen.hwMap);
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveFieldCentric(double x, double y, double rx) {
        motors[0].setPower(-1 * (y + x + rx));
        motors[1].setPower(-1 * (y - x - rx));
        motors[2].setPower(-1 * (y - x + rx));
        motors[3].setPower(-1 * (y + x - rx));
    }

    public void toggleNormalDrive() {
        if (!isNormalDriveEnabled())
            enableNormalDrive();
        else
            disableNormalDrive();
    }

    public void enableNormalDrive() {
        currentThrottle = normalThrottle;
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
        currentThrottle = normalThrottle;
        currentTurnThrottle = turnThrottle;
    }

    public boolean isSnailDriveEnabled() {
        return currentThrottle == snailDrive;
    }

    public boolean isNormalDriveEnabled() {
        return currentThrottle == normalThrottle;
    }

    public void brake() {
        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reverseAll() {
        for (DcMotorEx motor : motors)
            motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPower(double power) {
        for (DcMotorEx motor : motors)
            motor.setPower(power);
    }

    public static void setTurnThrottle(double turnThrottle) {
        currentThrottle = turnThrottle;
    }

    public double getThrottle() {
        return currentThrottle;
    }

    public void turn(double angle) {
        mecanumDrive.turn(angle);
    }
}