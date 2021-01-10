package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HardwareGenesis {
    public HardwareMap hwMap;
    public Servo flap, claw, arm, intakeStopper, flicker;
    public DcMotor leftIntake, rightIntake;
    public Motor flywheelFront, flywheelBack;
    public DcMotorEx[] drivetrainMotors;
    public BNO055IMU imu;

    public static String
            flapName = "flap",
            clawName = "claw",
            armName = "wobble",
            intakeStopperName = "intakeStopper",
            leftIntakeName = "leftIntake",
            rightIntakeName = "rightIntake",
            flickerName = "flicker",
            flywheelFrontName = "flywheelFront",
            flywheelBackName = "flywheelBack",
            imuName = "imu";

    public static String[] drivetrainNames = {"lowerRight", "lowerLeft", "upperRight", "upperLeft"};

    public HardwareGenesis(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drivetrainMotors = new DcMotorEx[4];
        flap = hwMap.servo.get(flapName);
        claw = hwMap.servo.get(clawName);
        arm = hwMap.servo.get(armName);
        intakeStopper = hwMap.servo.get(intakeStopperName);
        flicker = hwMap.servo.get(flickerName);

        leftIntake = hwMap.dcMotor.get(leftIntakeName);
        rightIntake = hwMap.dcMotor.get(rightIntakeName);

        flywheelFront = new Motor(hwMap, flywheelFrontName, Motor.GoBILDA.BARE);
        flywheelBack = new Motor(hwMap, flywheelBackName, Motor.GoBILDA.BARE);

        imu = hwMap.get(BNO055IMU.class, imuName);

        for (int i = 0; i < 4; i++) {
            drivetrainMotors[i] = hwMap.get(DcMotorEx.class, drivetrainNames[i]);
        }
    }
}
