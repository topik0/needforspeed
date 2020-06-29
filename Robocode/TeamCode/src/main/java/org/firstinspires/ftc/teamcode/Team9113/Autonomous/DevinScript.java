package org.firstinspires.ftc.teamcode.Team9113.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Team9113.Robot;

@Autonomous(name = "DevinScript", group = "Linear Opmode")
public class DevinScript extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, false);
        VueCV vuecv = new VueCV(hardwareMap);
        // Initializes the robot
        robot.startPositions();
        while(!isStarted()) {
            vuecv.updatePosition();
            telemetry.addData("Block Position: ", vuecv.getBlockPosition());
            telemetry.update();
        }

        waitForStart();
        // Write the actions here

        if(vuecv.block(0))
        {
            // Do something (1st block)
        } else if (vuecv.block(1))
        {
            // Do something (2nd block)

        } else if (vuecv.block(2))
        {
            // Do something (3rd block)
        }


    }
}
