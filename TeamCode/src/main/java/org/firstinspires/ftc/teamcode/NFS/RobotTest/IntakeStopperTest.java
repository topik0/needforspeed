package org.firstinspires.ftc.teamcode.NFS.RobotTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NFS.Control.Omnipad;
import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is a test OP mode for testing the flicker
 */
@Config
@TeleOp(name = "IntakeStopperTest", group = "Linear Opmode")
public class IntakeStopperTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        Omnipad pad = new Omnipad(gamepad1, gamepad2, robot);
        robot.intake.stopperStartPosition();
        waitForStart();
        while (opModeIsActive()) {
            if (pad.intakeStopperToggle()) {
                if (robot.intake.isUp())
                    robot.intake.down();
                else robot.intake.stopper.setPosition(1);
            }
            telemetry.addData("Intake Stopper State", robot.intake.getVerticalState());
            telemetry.update();
        }
    }
}