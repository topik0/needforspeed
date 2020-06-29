package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;

@Autonomous(name = "VisionAutoBlueTwo", group = "Linear Opmode")
public class VisionAutoBlueTwo extends LinearOpMode {

    // Initialize motor/servo variables

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        VueCV vuecv = new VueCV(hardwareMap);
        robot.startPositions();
        while(!isStarted()) {
            vuecv.updatePosition();
            telemetry.addData("Block Position: ", vuecv.getBlockPosition());
            telemetry.update();
        }
        waitForStart();


        if(vuecv.block(0)){
            robot.moveForward(600, 1);
            robot.moveRight(2650, .8);
            robot.intakeIn();
            robot.moveForward(500, .2);
            sleep(50);
            robot.intakeStop();
            robot.jointServo(1);
            robot.moveLeft(1200, .69);
            robot.motorLock();
            robot.moveBackward(5900, 1);
            robot.turnRight(2800, .8);
            //turnLeft(-1450, .5);
            robot.moveBackward(600, .69);
            sleep(100);
            robot.platformToggle();
            robot.placeBlock(0);
            robot.moveForward(2100, 1);
            robot.platformToggle();
            robot.moveLeft(2020, 1);
            robot.moveBackward(1100, 1);
            robot.moveRight(1500,1);
            robot.moveBackward(300, 1);
            robot.moveLeft(1776, 1);
        }
        if(vuecv.block(1)){
            robot.moveBackward(1000, 1);
            robot.moveRight(2650, .8);
            robot.intakeIn();
            robot.moveForward(500, .2);
            sleep(50);
            robot.intakeStop();
            robot.jointServo(1);
            robot.moveLeft(1200, .69);
            robot.motorLock();
            robot.moveBackward(4500, 1);
            robot.turnRight(2800, .8);
            //turnLeft(-1450, .5);
            robot.moveBackward(600, .69);
            sleep(100);
            robot.platformToggle();
            robot.placeBlock(0);
            robot.moveForward(2100, 1);
            robot.platformToggle();
            robot.moveLeft(2020, 1);
            robot.moveBackward(1100, 1);
            robot.moveRight(1500,1);
            robot.moveBackward(300, 1);
            robot.moveLeft(1776, 1);
        }
        if(vuecv.block(2)){
            robot.moveBackward(200, 1);
            robot.moveRight(2650, .8);
            robot.intakeIn();
            robot.moveForward(500, .2);
            sleep(50);
            robot.intakeStop();
            robot.jointServo(1);
            robot.moveLeft(1200, .69);
            robot.motorLock();
            robot.moveBackward(5350, 1);
            robot.turnRight(2800, .8);
            //turnLeft(-1450, .5);
            robot.moveBackward(600, .69);
            sleep(100);
            robot.platformToggle();
            robot.placeBlock(0);
            robot.moveForward(2100, 1);
            robot.platformToggle();
            robot.moveLeft(2020, 1);
            robot.moveBackward(1100, 1);
            robot.moveRight(1500,1);
            robot.moveBackward(300, 1);
            robot.moveLeft(1776, 1);
        }
        vuecv.terminate();
        sleep(10000);
    }
}
