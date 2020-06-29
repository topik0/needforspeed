package org.firstinspires.ftc.teamcode.Team9113;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SlideMove", group = "Linear Opmode")
public class SlideMove extends LinearOpMode {
    private double[] milliTime = new double[8];

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        robot.startPositions();
        // Initialize variables
        boolean capstoneArmUp = true;
        boolean intakeStopped = true;
        boolean jointUp = true;
        boolean clawUp = true;
        boolean goingIn = true;
        boolean platformDown = false;
        boolean capstonePlaced = false;
        double leftPower, rightPower;
        final int timeThreshold = 250;
        final double intakeSpeed = .35;
        // Initialize components
        waitForStart();
        //  telemetry.update();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                robot.slideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slideMotorOne.setPower(1);
                robot.intakeIn();
            }
        }
    }

}