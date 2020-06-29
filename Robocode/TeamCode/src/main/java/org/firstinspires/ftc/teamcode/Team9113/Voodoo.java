package org.firstinspires.ftc.teamcode.Team9113;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    private double[] milliTime = new double[9];
    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        // Set things to starting positions
        if (opModeIsActive())
        robot.startPositions();
        // Initialize variables
        double leftPower, rightPower;
        final int timeThreshold = 350;
        waitForStart();
            while (opModeIsActive()) {
                robot.drivetrain.setLeftPower(gamepad1.left_stick_y);
                robot.drivetrain.setRightPower(gamepad1.right_stick_y);
                if (gamepad1.dpad_left) {
                    robot.motorThree.setPower(-robot.strafeSpeed);
                    robot.motorOne.setPower(robot.strafeSpeed);
                    robot.motorTwo.setPower(robot.strafeSpeed);
                    robot.motorZero.setPower(-robot.strafeSpeed);
                } else if (gamepad1.dpad_right) {
                    robot.motorThree.setPower(robot.strafeSpeed);
                    robot.motorOne.setPower(-robot.strafeSpeed);
                    robot.motorTwo.setPower(-robot.strafeSpeed);
                    robot.motorZero.setPower(robot.strafeSpeed);
                } else if (gamepad1.dpad_up) {
                    robot.motorThree.setPower(-robot.strafeSpeed);
                    robot.motorOne.setPower(-robot.strafeSpeed);
                    robot.motorTwo.setPower(-robot.strafeSpeed);
                    robot.motorZero.setPower(-robot.strafeSpeed);
                } else if (gamepad1.dpad_down) {
                    robot.motorThree.setPower(robot.strafeSpeed);
                    robot.motorOne.setPower(robot.strafeSpeed);
                    robot.motorTwo.setPower(robot.strafeSpeed);
                    robot.motorZero.setPower(robot.strafeSpeed);
                }
                if(gamepad2.right_bumper && System.currentTimeMillis()-milliTime[8] > timeThreshold) {
                    robot.snailDriveToggle();
                    stopwatch(9);
                }
                if(gamepad1.left_bumper && System.currentTimeMillis()-milliTime[2] > timeThreshold){
                    robot.toggleHyperDrive();
                    stopwatch(3);
                }
                telemetry.addData("", "Slide Position: " + robot.slideMotorOne.getCurrentPosition());
                telemetry.addData("", "Hyperdrive: " + robot.hyperDriveEngaged());
                telemetry.addData("", "Snaildrive: ", robot.snailDriveEngaged());
                telemetry.update();
            }
    }
    private void stopwatch(int type){
        if(type == 1)
            milliTime[0] = System.currentTimeMillis();
        if(type == 2)
            milliTime[1] = System.currentTimeMillis();
        if(type == 3)
            milliTime[2] = System.currentTimeMillis();
        if(type == 4)
            milliTime[3] = System.currentTimeMillis();
        if(type == 5)
            milliTime[4] = System.currentTimeMillis();
        if(type == 6)
            milliTime[5] = System.currentTimeMillis();
        if(type == 7)
            milliTime[6] = System.currentTimeMillis();
        if(type == 8)
            milliTime[7] = System.currentTimeMillis();
        if(type == 9)
            milliTime[8] = System.currentTimeMillis();
    }
}
