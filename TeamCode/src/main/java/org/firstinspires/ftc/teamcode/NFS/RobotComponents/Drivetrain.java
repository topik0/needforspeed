package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

@Config
public class Drivetrain {
    /**
     * upperLeft [0] upperRight [1]
     * lowerLeft [2] lowerRight [3]
     */

    private Robot robot;
    private BNO055IMU imu;

    public static double normalThrottle = 1;
    public static double snailDrive = .65;
    public static double turnThrottle = .75;
    public static double snailTurnThrottle = .4;
    public static double currentThrottle = normalThrottle;
    public static double currentTurnThrottle = turnThrottle;
    public static double turnkP = .04, turnErrorThreshold = 2;

    public DcMotorEx[] motors;
    public SampleMecanumDrive mecanumDrive;

    public Drivetrain(HardwareGenesis gen, Robot robot) {
        this.robot = robot;
        imu = robot.genesis.imu;
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

    public void leftPower(double power) {
        motors[0].setPower(power);
        motors[2].setPower(power);
    }

    public void rightPower(double power) {
        motors[1].setPower(power);
        motors[3].setPower(power);
    }

    public void setSideMotorPowers(double leftPower, double rightPower) {
        leftPower(leftPower);
        rightPower(rightPower);
    }

    public void turn(double degrees) {
        double error = -1 * degrees;
        double originalHeading = getHeading();
        double leftPow;
        double rightPow;
        while ((Math.abs(error) >= turnErrorThreshold)) {
            robot.flywheels.run();
            error = getHeading() - degrees - originalHeading;
            leftPow = error * turnkP;
            rightPow = -error * turnkP;
            setSideMotorPowers(leftPow, rightPow);
        }
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}