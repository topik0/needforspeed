package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
  private double[] milliTime = new double[9];

  @Override
  public void runOpMode() {
    Robot robot = new Robot(hardwareMap);
    // Set things to starting positions
    if (opModeIsActive()) robot.startPositions();
    // Initialize variables
    final int timeThreshold = 350;
    waitForStart();
    while (opModeIsActive()) {
      robot.drivetrain.setLeftPower(gamepad1.left_stick_y);
      robot.drivetrain.setRightPower(gamepad1.right_stick_y);
      if (gamepad1.dpad_left) {
        robot.drivetrain.moveLeft();
      } else if (gamepad1.dpad_right) {
        robot.drivetrain.moveRight();
      } else if (gamepad1.dpad_up) {
        robot.drivetrain.driveForward();
      } else if (gamepad1.dpad_down) {
        robot.drivetrain.driveBackward();
      }
      if (gamepad2.right_bumper && System.currentTimeMillis() - milliTime[8] > timeThreshold) {
        robot.drivetrain.toggleSnailDrive();
        stopwatch(9);
      }
      if (gamepad1.left_bumper && System.currentTimeMillis() - milliTime[2] > timeThreshold) {
        robot.drivetrain.toggleHyperdrive();
        stopwatch(3);
      }
      telemetry.addData("", "Hyperdrive: " + robot.drivetrain.isHyperdriveEnabled());
      telemetry.addData("", "Snaildrive: ", robot.drivetrain.isSnailDriveEnabled());
      telemetry.update();
    }
  }

  private void stopwatch(int type) {
    if (type == 1) milliTime[0] = System.currentTimeMillis();
    if (type == 2) milliTime[1] = System.currentTimeMillis();
    if (type == 3) milliTime[2] = System.currentTimeMillis();
    if (type == 4) milliTime[3] = System.currentTimeMillis();
    if (type == 5) milliTime[4] = System.currentTimeMillis();
    if (type == 6) milliTime[5] = System.currentTimeMillis();
    if (type == 7) milliTime[6] = System.currentTimeMillis();
    if (type == 8) milliTime[7] = System.currentTimeMillis();
    if (type == 9) milliTime[8] = System.currentTimeMillis();
  }
}
