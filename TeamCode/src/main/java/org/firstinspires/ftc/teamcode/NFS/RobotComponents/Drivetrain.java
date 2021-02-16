package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Exceptions.BadInitializationException;
import org.firstinspires.ftc.teamcode.NFS.TeleOP.Voodoo;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class contains the drivetrain and drive control
 */
@Config
public class Drivetrain {
    /*
     * Map of motors
     * frontLeft [1] frontRight [2]
     * backLeft [0] backRight [3]
     */

    /**
     * The default drivetrain throttle
     */
    public static double normalThrottle = 1;
    /**
     * The default throttle for turning
     */
    public static double turnThrottle = Voodoo.normalTurnThrottle;
    /**
     * The kP and error thresholds for the turn function
     */
    public static double turnKP = .04, turnErrorThreshold = 2;

    /**
     * The SampleMecanumDrive used for odometry and Road Runner
     */
    public SampleMecanumDrive mecanumDrive;

    /**
     * The drivetrain motors
     */
    private final DcMotorEx[] motors;

    private final Robot robot;
    private final BNO055IMU imu;
    private double currentThrottle = normalThrottle;
    private double currentTurnThrottle = turnThrottle;

    /**
     * Drivetrain constructor
     *
     * @param gen   the HardwareGenesis object needed to get the drivetrain motors and IMU
     * @param robot the Robot object needed for Robot functions
     */
    public Drivetrain(HardwareGenesis gen, Robot robot) {
        if (gen == null)
            throw new BadInitializationException("Null Genesis detected in drivetrain");
        imu = gen.imu;
        motors = gen.drivetrainMotors;
        if (motors == null)
            throw new BadInitializationException("Null motors detected in drivetrain");
        if (imu == null)
            throw new BadInitializationException("Null IMU detected in drivetrain");
        this.robot = robot;
        if (robot == null)
            throw new BadInitializationException("Null Robot detected in drivetrain");
        mecanumDrive = new SampleMecanumDrive(gen.hwMap);
        getBackLeft().setDirection(DcMotor.Direction.REVERSE);
        getFrontLeft().setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Drives the drivetrain in field centric mode
     *
     * @param x  the x value
     * @param y  the y value
     * @param rx the right x value
     */
    public void driveFieldCentric(double x, double y, double rx) {
        getFrontLeft().setPower(1 * (y + x + rx));
        getFrontRight().setPower(1 * (y - x - rx));
        getBackLeft().setPower(1 * (y - x + rx));
        getBackRight().setPower(1 * (y + x - rx));
    }

    /**
     * Brakes the drivetrain motors using ZeroPowerBehavior
     */
    public void brake() {
        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Reverses all of the drivetrain motors
     */
    public void reverseAll() {
        for (DcMotorEx motor : motors)
            motor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Sets the power of all drivetrain motors
     *
     * @param power the power to be set
     */
    public void setPower(double power) {
        for (DcMotorEx motor : motors)
            motor.setPower(power);
    }

    /**
     * Sets the power of the left side of the drivetrain
     *
     * @param power the power to be set
     */
    public void leftPower(double power) {
        getBackLeft().setPower(power);
        getFrontLeft().setPower(power);
    }

    /**
     * Sets the power of the right side of the drivetrain
     *
     * @param power the power to be set
     */
    public void rightPower(double power) {
        getBackRight().setPower(power);
        getFrontRight().setPower(power);
    }

    /**
     * Sets the power of both sides of the drivetrain
     *
     * @param leftPower  the power to be given to the left side
     * @param rightPower the power to be given to the right side
     */
    public void setSideMotorPowers(double leftPower, double rightPower) {
        leftPower(leftPower);
        rightPower(rightPower);
    }

    /**
     * Turns the drivetrain a specified angle in degrees
     *
     * @param degrees the angle to turn in degrees
     */
    public void turn(double degrees) {
        double error = -1 * degrees;
        double originalHeading = getHeading();
        double leftPow;
        double rightPow;
        while ((Math.abs(error) >= turnErrorThreshold)) {
            robot.flywheels.run();
            error = getHeading() - degrees - originalHeading;
            leftPow = error * turnKP;
            rightPow = -error * turnKP;
            setSideMotorPowers(leftPow, rightPow);
        }
    }

    /**
     * Turns the drivetrain to specified angle in degrees
     *
     * @param degrees the angle to turn in degrees
     */

    public void turnTo(double degrees) {
        double adjustedDegrees = degrees + Voodoo.heading;
        double error = -1 * adjustedDegrees;
        double leftPow;
        double rightPow;
        while ((Math.abs(error) >= turnErrorThreshold)) {
            robot.flywheels.run();
            error = getHeading() - adjustedDegrees;
            leftPow = error * turnKP;
            rightPow = -error * turnKP;
            setSideMotorPowers(leftPow, rightPow);
        }
    }


    /**
     * Gets the current heading using the IMU
     *
     * @return the current heading
     */
    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Sets the turn throttle of the drivetrain
     *
     * @param turnThrottle the turn throttle to be set
     */
    public void setTurnThrottle(double turnThrottle) {
        currentTurnThrottle = turnThrottle;
    }

    /**
     * Sets the throttle of the drivetrain
     *
     * @param throttle the throttle to be set
     */
    public void setDrivetrainThrottle(double throttle) {
        currentThrottle = throttle;
    }

    /**
     * Gets the current throttle
     *
     * @return the current throttle
     */
    public double getThrottle() {
        return currentThrottle;
    }

    /**
     * Gets the current turn throttle
     *
     * @return the turn throttle
     */
    public double getTurnThrottle() {
        return currentTurnThrottle;
    }

    public DcMotorEx getBackLeft() {
        return motors[0];
    }

    public DcMotorEx getFrontLeft() {
        return motors[1];
    }

    public DcMotorEx getFrontRight() {
        return motors[2];
    }

    public DcMotorEx getBackRight() {
        return motors[3];
    }

    public DcMotorEx[] getMotorsArray() {
        return motors;
    }
}