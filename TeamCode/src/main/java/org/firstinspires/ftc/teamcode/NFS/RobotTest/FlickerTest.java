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
@TeleOp(name = "FlickerTest", group = "Linear Opmode")
public class FlickerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        Omnipad pad = new Omnipad(gamepad1, gamepad2, robot);
        waitForStart();
        while (opModeIsActive()) {
            robot.flywheels.run();
            robot.flicker.checkState();
            if (pad.shootRing()) robot.flicker.shootOut();
            if (pad.flywheelsToggle()) robot.flywheels.togglePID();
            if (pad.shootRingNonFSM()) robot.flicker.launch();
            telemetry.addData("Flicker State", robot.flicker.getState());
            telemetry.addData("Flywheel State", robot.flywheels.getRunState());
            telemetry.update();
        }
    }
}