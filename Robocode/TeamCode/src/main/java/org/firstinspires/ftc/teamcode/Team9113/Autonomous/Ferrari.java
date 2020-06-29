package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;


@Autonomous(name = "Ferrari", group = "Linear Opmode")
public class Ferrari extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, false);
        robot.startPositions();
        waitForStart();
        robot.moveRight(1800, .8);
        sleep(500);
        for(int i = 1, add; i < 7; i++) {
            add = i*600;
            sleep(1000);
            robot.grabberGrabMotionOne();
            sleep(500);
            robot.moveLeft(3000, .8);
            sleep(500);
            robot.moveBackward(500+add, .8);
            sleep(1500);
            robot.grabberPlaceMotion();
            sleep(500);
            robot.moveForward(1800+add, .8);
            sleep(1000);
            robot.moveRight(1000, .8);
            sleep(1000);
            robot.grabCycle(i);
        }
    }
}
