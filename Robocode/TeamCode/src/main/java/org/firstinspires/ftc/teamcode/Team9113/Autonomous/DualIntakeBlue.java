package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;


@Autonomous(name = "DualIntakeBlue", group = "Linear Opmode")
public class DualIntakeBlue extends LinearOpMode {
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
            robot.resetAngle();
            robot.moveForward(150, speed);
            robot.moveRight(1800, speed);
            sleep(250);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveLeft(400, speed);
            sleep(250);
            robot.moveBackward(5500, -speed);
            sleep(250);
            robot.moveRight(750, speed);
            sleep(500);
            robot.grabberPlaceMotion();
            robot.turn90('r');
            robot.moveBackward(500, .5);
            sleep(500);
            robot.platformServosDown();
            sleep(250);
            robot.moveForward(2000, speed);
            robot.turnFoundation90('l');
            robot.moveBackward(1000, speed);
            robot.platformServosUp();
            sleep(250);
            robot.moveForward(100, speed);
            robot.moveRight(1500, speed);
            robot.moveBackward(750, speed);
            robot.grabberArmStart();
            robot.grabberElbowStart();
            robot.moveForward(2350, speed);
        } else if (vuecv.block(1))
        {
            vuecv.terminate();
            robot.resetAngle();
            robot.moveBackward(350, speed);
            robot.moveRight(1820, speed);
            sleep(250);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveLeft(400, speed);
            sleep(250);
            robot.moveBackward(5000, -speed);
            sleep(250);
            robot.moveRight(650, speed);
            sleep(500);
            robot.grabberPlaceMotion();
            robot.turn90('r');
            robot.moveBackward(500, .5);
            sleep(500);
            robot.platformServosDown();
            sleep(150);
            robot.moveForward(2000, speed);
            robot.turnFoundation90('l');
            robot.moveBackward(1000, speed);
            robot.platformServosUp();
            sleep(250);
            robot.moveForward(100, speed);
            robot.moveRight(1500, speed);
            robot.moveBackward(750, speed);
            robot.grabberArmStart();
            robot.grabberElbowStart();
            robot.moveForward(2350, speed);
        } else if (vuecv.block(2))
        {
            vuecv.terminate();
            robot.resetAngle();
            robot.moveBackward(600, speed);
            robot.moveRight(1800, speed);
            sleep(250);
            robot.grabberGrabMotionOne();
            sleep(250);
            robot.moveLeft(400, speed);
            sleep(250);
            robot.moveBackward(4900, -speed);
            sleep(250);
            robot.moveRight(750, speed);
            sleep(500);
            robot.grabberPlaceMotion();
            robot.moveBackward(550, speed);
            robot.turn90('r');
            robot.moveBackward(500, .5);
            sleep(500);
            robot.platformServosDown();
            sleep(150);
            robot.moveForward(2000, speed);
            robot.turnFoundation90('l');
            robot.moveBackward(1000, speed);
            robot.platformServosUp();
            sleep(250);
            robot.moveForward(100, speed);
            robot.moveRight(1776, speed);
            robot.moveBackward(750, speed);
            robot.grabberArmStart();
            robot.grabberElbowStart();
            robot.moveForward(2350, speed);
        }
    }
}
