package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class contains all of the hardware objects needed for the robot
 */
@Config
public class HardwareGenesis {
    /**
     * The hardware map needed to access the robot hardware
     */
    public HardwareMap hwMap;
    /**
     * The servos used on the robot
     */
    public Servo flap, claw, arm, intakeStopper, flicker;
    /**
     * The intake motors
     */
    public DcMotor leftIntake, rightIntake;
    /**
     * The flywheel motors
     */
    public Motor flywheelFront, flywheelBack;
    /**
     * The drivetrain motors
     */
    public DcMotorEx[] drivetrainMotors;
    /**
     * The IMU
     */
    public BNO055IMU imu;

    /**
     * Names of all of the robot hardware in the configuration
     */
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

    /**
     * Names of the drivetrain motors in the configuration
     */
    public static String[] drivetrainNames = {"backLeft", "frontLeft", "frontRight", "backRight"};

    /**
     * HardwareGenesis constructor which makes all of the robot hardware objects
     *
     * @param hwMap the HardwareMap needed to access robot components
     */
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
