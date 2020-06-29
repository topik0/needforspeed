package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Team9113.Robot;

@Autonomous(name = "AutoPanelBlue2", group = "_works")
public class AutoPanelBlue2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        robot.startPositions();
        waitForStart();

        // goes to platform
        robot.platformServosUp();
        sleep(500);
        robot.moveLeft(500, .8);
        sleep(500);

        robot.moveBackward(1860, .5);
        sleep(500);

        robot.platformServosDown();
        sleep(500);

        // pulls platform back
        robot.moveForward(1860, .5);
        sleep(500);

        robot.platformServosUp();
        sleep(500);

        // goes under the bridge
        robot.moveRight(3000, .8);
        sleep(500);
    }
}
