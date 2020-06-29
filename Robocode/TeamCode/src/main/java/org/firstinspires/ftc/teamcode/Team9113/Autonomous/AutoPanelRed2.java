package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team9113.Robot;

@Autonomous(name = "AutoPanelRed2", group = "_works")
public class AutoPanelRed2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        robot.startPositions();
        waitForStart();

        // goes to platform
        robot.platformServosUp();
        sleep(500);
        robot.moveRight(750, .8);
        sleep(500);

        robot.moveBackward(1860, .5);
        sleep(500);

        robot.platformServosDown();
        sleep(500);

        // pulls platform back
        robot.moveForward(1900, .5);
        sleep(500);

        robot.platformServosUp();
        sleep(500);

        // goes under the bridge
        robot.moveLeft(3250, .8);
        sleep(500);
    }
}
