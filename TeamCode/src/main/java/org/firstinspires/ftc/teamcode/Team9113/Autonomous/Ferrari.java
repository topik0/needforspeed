package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Team9113.Robot;

@Autonomous(name = "Ferrari", group = "Linear Opmode")
public class Ferrari extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        waitForStart();
    }
}