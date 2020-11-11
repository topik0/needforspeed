package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    /**
     * upperLeft [0] upperRight [1]
     * lowerLeft [2] lowerRight [3]
     */
    private final double setHyperdriveThrottle = 1.0;

    private final double setStrafeHyperdriveThrottle = 1.0;
    private final double setBirdThrottle = .7;
    private final double setStrafeBirdThrottle = .7;
    private final double setSnailStrafeThrottle = .37;
    private final double setSnailThrottle = .37;
    private double currentThrottle = setBirdThrottle;
    private double currentStrafeThrottle = setStrafeBirdThrottle;

    public DcMotor[] drivetrain = new DcMotor[4];
    protected HardwareMap hwMap;

    public Drivetrain(HardwareMap hwMap) {
        this.hwMap = hwMap;
        for (int i = 0; i < 4; i++) {
            String[] drivetrainNames = {"upperLeft", "upperRight", "lowerLeft", "lowerRight"};
            drivetrain[i] = this.hwMap.dcMotor.get(drivetrainNames[i]);
        }
        drivetrain[1].setDirection(DcMotor.Direction.REVERSE);
        drivetrain[3].setDirection(DcMotor.Direction.REVERSE);
    }

    public void moveLeft() {
        drivetrain[0].setPower(-currentStrafeThrottle);
        drivetrain[1].setPower(currentStrafeThrottle);
        drivetrain[2].setPower(currentStrafeThrottle);
        drivetrain[3].setPower(-currentStrafeThrottle);
    }

    public void moveRight() {
        drivetrain[0].setPower(-currentStrafeThrottle);
        drivetrain[1].setPower(currentStrafeThrottle);
        drivetrain[2].setPower(currentStrafeThrottle);
        drivetrain[3].setPower(-currentStrafeThrottle);
    }

    public void driveForward() {
        for (int i = 0; i < 4; i++) {
            drivetrain[i].setPower(-currentThrottle);
        }
    }

    public void driveBackward() {
        for (int i = 0; i < 4; i++) {
            drivetrain[i].setPower(currentThrottle);
        }
    }

    public void toggleHyperdrive() {
        if (!isHyperdriveEnabled()) {
            enableHyperdrive();
            return;
        }
        disableHyperdrive();
    }

    public void enableHyperdrive() {
        currentThrottle = setHyperdriveThrottle;
        currentStrafeThrottle = setStrafeHyperdriveThrottle;
    }

    public void disableHyperdrive() {
        currentThrottle = setSnailThrottle;
        currentStrafeThrottle = setSnailStrafeThrottle;
    }

    public void toggleSnailDrive() {
        if (!isSnailDriveEnabled()) {
            enableSnailDrive();
        }
    }

    public void enableSnailDrive() {
        currentThrottle = setSnailThrottle;
        currentStrafeThrottle = setSnailStrafeThrottle;
    }

    public void disableSnailDrive() {
        currentThrottle = setBirdThrottle;
        currentStrafeThrottle = setStrafeBirdThrottle;
    }

    public boolean isSnailDriveEnabled() {
        return currentThrottle == setSnailThrottle && currentStrafeThrottle == setSnailStrafeThrottle;
    }

    public boolean isHyperdriveEnabled() {
        return currentThrottle == setHyperdriveThrottle
                && currentStrafeThrottle == setStrafeHyperdriveThrottle;
    }

    public void brakeMotors() {
        for (DcMotor motor : drivetrain)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFieldCentricPower(double y, double x, double rx) {
        drivetrain[2].setPower(y - x + rx);
        drivetrain[0].setPower(y + x + rx);
        drivetrain[1].setPower(y - x - rx);
        drivetrain[3].setPower(y + x - rx);
    }

    public void setPower(double power) {
        for (int i = 0; i < 4; i++) {
            drivetrain[i].setPower(power);
        }
    }

    public void setLeftPower(double power) {
        for (int i = 0; i <= 2; i += 2) {
            drivetrain[i].setPower(power);
        }
    }

    public void setRightPower(double power) {
        for (int i = 1; i <= 3; i += 2) {
            drivetrain[i].setPower(power);
        }
    }

    public double getThrottle() {
        return currentThrottle;
    }

    public double getStrafeThrottle() {
        return currentStrafeThrottle;
    }
}
