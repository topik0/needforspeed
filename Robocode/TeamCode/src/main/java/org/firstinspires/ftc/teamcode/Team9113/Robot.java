package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot extends LinearOpMode {
    /*
    Robot constructor
     */
    protected HardwareMap hwMap;
    public Drivetrain drivetrain;
    public Servo flap, intakeStopper, flicker, claw;
    public DcMotor flywheelFront, flyWheelBack;
    public RobotPreferences pref;

    public Robot(HardwareMap hwMap) {
        this.hwMap = hwMap;
        drivetrain = new Drivetrain(hwMap);
        // pref = new RobotPreferences();
        flap = this.hwMap.servo.get("flap");
        intakeStopper = this.hwMap.servo.get("intakeStopper");
        flicker = this.hwMap.servo.get("flicker");
        claw = this.hwMap.servo.get("claw");
        flywheelFront = this.hwMap.dcMotor.get("flywheelFront");
        flyWheelBack = this.hwMap.dcMotor.get("flywheelBack");
    }

    /*
    Initializes the robot and the components
     */
    public void initHardware() {
    }

    public void startPositions() {
    }

    public void setFlywheelPower(double power) {
        flywheelFront.setPower(power);
        flyWheelBack.setPower(power);
    }

    public void startFlywheels(){
        setFlywheelPower(1);
    }

    public void stopFlywheels(){
        setFlywheelPower(0);
    }

    public void shootDisc() {
        flicker.setPosition(.6);
        sleep(100);
        flicker.setPosition(.3);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
