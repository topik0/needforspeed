package org.firstinspires.ftc.teamcode.NFS.TeleOP;

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
                robot.drivetrain.motors[3].setPower(1);
            if (gamepad1.b)
                robot.drivetrain.motors[2].setPower(1);
            if (gamepad1.x)
                robot.drivetrain.motors[1].setPower(1);
            if (gamepad1.y)
                robot.drivetrain.motors[0].setPower(1);
        }
    }
}