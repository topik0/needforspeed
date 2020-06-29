package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;


@Autonomous(name = "Move Auto", group = "Linear Opmode")
public class MoveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        VueCV vuecv = new VueCV(hardwareMap);
        double speed = 1.0;
        double strafeSpeed = .6;
        robot.startPositions();

        waitForStart();
        robot.moveForward(1000, 1);
    }
}
