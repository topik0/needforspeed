package org.firstinspires.ftc.teamcode.Team9113;

import com.arcrobotics.ftclib.hardware.motors.Motor;
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

    public Motor[] drivetrain = new Motor[4];
    protected HardwareMap hwMap;

    public Drivetrain(HardwareMap hwMap) {
        this.hwMap = hwMap;
        for (int i = 0; i < 4; i++) {
            String[] drivetrainNames = {"upperLeft", "upperRight", "lowerLeft", "lowerRight"};
            drivetrain[i] = new Motor(this.hwMap, drivetrainNames[i], Motor.GoBILDA.RPM_312);
            drivetrain[i].setRunMode(Motor.RunMode.RawPower);
        }
        drivetrain[3].setInverted(true);
    }

    public void moveLeft() {
        drivetrain[0].set(-currentStrafeThrottle);
        drivetrain[1].set(currentStrafeThrottle);
        drivetrain[2].set(currentStrafeThrottle);
        drivetrain[3].set(-currentStrafeThrottle);
    }

    public void moveRight() {
        drivetrain[0].set(-currentStrafeThrottle);
        drivetrain[1].set(currentStrafeThrottle);
        drivetrain[2].set(currentStrafeThrottle);
        drivetrain[3].set(-currentStrafeThrottle);
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

    public double getStrafeThrottle() {
        return currentStrafeThrottle;
    }
}
