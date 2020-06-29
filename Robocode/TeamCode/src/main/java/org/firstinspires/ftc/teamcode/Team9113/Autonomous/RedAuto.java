package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;

@Autonomous(name = "RedAuto", group = "Linear Opmode")
public class RedAuto extends LinearOpMode {
    // Initialize variables

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        VueCV vuecv = new VueCV(hardwareMap);
        // Assign motor/servo values
        vuecv.updatePosition();
        robot.startPositions();

        // Start camera
        while(!isStarted()) {
            vuecv.updatePosition();
            telemetry.addData("Block Position: ", vuecv.getBlockPosition());
            telemetry.update();
        }
        waitForStart();

        if(vuecv.block(0)){
            robot.moveBackward(600, 1);
            robot.moveRight(1500, .8);
            robot.turnRight(2800, .8);
            robot.moveLeft(1150, .69);
            robot.intakeIn();
            robot.moveForward(500, .2);
            sleep(50);
            robot.intakeStop();
            robot.jointServo(1);
            robot.moveRight(1200, .69);
            robot.motorLock();
            robot.moveBackward(5900, 1);
            robot.turnRight(-1450, .5);
            robot.moveBackward(600, .69);
            sleep(100);
            robot.platformToggle();
            robot.placeBlock(0);
            robot.moveForward(2200, 1);
            robot.platformToggle();
            robot.moveLeft(2020, 1);
            robot.moveBackward(1100, 1);
            robot.moveRight(1500,1);
            robot.moveBackward(300, 1);
            robot.moveLeft(1776, 1);
        }
        if(vuecv.block(1)){
            robot.moveForward(500, 1);
            robot.moveRight(1500, .8);
            robot.turnRight(2800, .8);
            robot.moveLeft(1150, .69);
            robot.intakeIn();
            robot.moveForward(500, .2);
            sleep(50);
            robot.intakeStop();
            robot.jointServo(1);
            robot.moveRight(1200, .69);
            robot.motorLock();
            robot.moveBackward(4500, 1);
            robot.turnRight(-1450, .5);
            robot.moveBackward(600, .69);
            sleep(100);
            robot.platformToggle();
            robot.placeBlock(0);
            robot.moveForward(2200, 1);
            robot.platformToggle();
            robot.moveLeft(2020, 1);
            robot.moveBackward(1100, 1);
            robot.moveRight(1500,1);
            robot.moveBackward(300, 1);
            robot.moveLeft(1776, 1);
        }
        if(vuecv.block(2)){
            robot.moveBackward(200, 1);
            robot.moveRight(1500, .8);
            robot.turnRight(2800, .8);
            robot.moveLeft(1150, .69);
            robot.intakeIn();
            robot.moveForward(500, .2);
            sleep(50);
            robot.intakeStop();
            robot.jointServo(1);
            robot.moveRight(1200, .69);
            robot.motorLock();
            robot.moveBackward(5350, 1);
            robot.turnRight(-1450, .5);
            robot.moveBackward(600, .69);
            sleep(100);
            robot.platformToggle();
            robot.placeBlock(0);
            robot.moveForward(2200, 1);
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
