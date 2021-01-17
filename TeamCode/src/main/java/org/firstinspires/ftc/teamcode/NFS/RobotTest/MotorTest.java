package org.firstinspires.ftc.teamcode.NFS.RobotTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is used to test the individual motors on the drivetrain
 */
@Config
@TeleOp(name = "MotorTest", group = "Linear Opmode")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a)
                robot.drivetrain.getFrontLeft().setPower(1);
            else robot.drivetrain.getFrontLeft().setPower(0);
            if (gamepad1.b)
                robot.drivetrain.getFrontRight().setPower(1);
            else robot.drivetrain.getFrontRight().setPower(0);
            if (gamepad1.x)
                robot.drivetrain.getBackLeft().setPower(1);
            else robot.drivetrain.getBackLeft().setPower(0);
            if (gamepad1.y)
                robot.drivetrain.getBackRight().setPower(1);
            else robot.drivetrain.getBackRight().setPower(0);
            telemetry.addData("Back Left", robot.drivetrain.getBackLeft().getCurrentPosition());
            telemetry.addData("Back Right", robot.drivetrain.getBackRight().getCurrentPosition());
            telemetry.addData("Front Left", robot.drivetrain.getFrontLeft().getCurrentPosition());
            telemetry.addData("Front Right", robot.drivetrain.getFrontRight().getCurrentPosition());
            telemetry.addData("Flywheels", robot.flywheels.flywheelFront.getCurrentPosition());

            telemetry.update();
        }
    }
}