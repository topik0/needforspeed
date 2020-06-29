package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;


@Autonomous(name = "TriBlockBlue", group = "Linear Opmode")
public class TriBlockBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        VueCV vuecv = new VueCV(hardwareMap);
        double speed = 1.0;
        double strafeSpeed = .6;
        robot.startPositions();
        while(!isStarted()) {
            vuecv.updatePosition();
            telemetry.addData("Block Position: ", vuecv.getBlockPosition());
            telemetry.update();
        }

        waitForStart();
        if(vuecv.block(0))
        {
            vuecv.terminate();
            telemetry.addData("Block sdfasdf: ", '2');
            telemetry.update();
            robot.moveForward(200, speed);
            robot.moveRight(1800, strafeSpeed);
            sleep(250);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveLeft(400, strafeSpeed);
            sleep(250);
            robot.moveBackward(4680, -speed);
            sleep(250);
            robot.moveRight(750, strafeSpeed);
            sleep(500);
            robot.grabberPlaceMotion();
            sleep(250);
            robot.moveLeft(500, strafeSpeed);
            robot.moveForward(6100, speed);
            robot.moveRight(275, strafeSpeed);
            sleep(50);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveBackward(6750, -speed);
            sleep(250);
            robot.moveRight(1100, strafeSpeed);
            sleep(250);
            robot.grabberPlaceMotion();
            sleep(250);
        } else if (vuecv.block(1))
        {
            vuecv.terminate();
            robot.resetAngle();
            robot.moveBackward(350, speed);
            robot.moveRight(1800, speed);
            sleep(250);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveLeft(400, speed);
            sleep(250);
            robot.moveBackward(4680, -speed);
            sleep(250);
            robot.moveRight(650, speed);
            sleep(500);
            robot.grabberPlaceMotion();
            sleep(250);
            robot.moveLeft(500, speed);
            robot.moveForward(6100, speed);
            robot.grabberGrabMotionTwo();
            sleep(250);
            robot.moveLeft(425, speed);
            robot.moveBackward(6750, -speed);
            sleep(250);
            robot.moveRight(675, speed);
            sleep(250);
            robot.grabberPlaceMotion();
            sleep(250);
            robot.turn90('r');
            robot.moveBackward(500, speed);
            robot.platformServosDown();
            sleep(150);
            robot.moveForward(500, speed);
            robot.turn90('l');
            robot.moveBackward(500, speed);
            robot.platformServosUp();
            robot.moveLeft(500, strafeSpeed);
            robot.moveForward(1000, speed);
        } else if (vuecv.block(2))
        {
            vuecv.terminate();
            telemetry.addData("Block sdfasdf: ", '2');
            telemetry.update();
            robot.moveBackward(680, speed);
            robot.moveRight(1800, speed);
            sleep(250);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveLeft(400, speed);
            sleep(250);
            robot.moveBackward(4680, -speed);
            sleep(250);
            robot.moveRight(750, speed);
            sleep(500);
            robot.grabberPlaceMotion();
            sleep(250);
            robot.moveLeft(500, speed);
            robot.moveForward(6100, speed);
            robot.moveRight(275, speed);
            sleep(50);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveBackward(6750, -speed);
            sleep(250);
            robot.moveRight(1100, speed);
            sleep(250);
            robot.grabberPlaceMotion();
            sleep(250);
        }
        vuecv.terminate();

    }
}
