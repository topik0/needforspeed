package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Team9113.Control.Superpad;

@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
  private double[] milliTime = new double[9];

  @Override
  public void runOpMode() {
    Robot robot = new Robot(hardwareMap);
    Superpad superpad = new Superpad(gamepad1, gamepad2);
    // Set things to starting positions
    if (opModeIsActive()) robot.startPositions();
    // Initialize variables
    final int timeThreshold = 350;
    waitForStart();
    while (opModeIsActive()) {
      robot.drivetrain.setLeftPower(superpad.leftJoystickY(0));
      robot.drivetrain.setRightPower(superpad.rightJoystickY(0));
      if (superpad.dpadLeft(0)) {
        robot.drivetrain.moveLeft();
      } else if (superpad.dpadLeft(0)) {
        robot.drivetrain.moveRight();
      } else if (superpad.dpadLeft(0)) {
        robot.drivetrain.driveForward();
      } else if (superpad.dpadLeft(0)) {
        robot.drivetrain.driveBackward();
      }
      if (superpad.rightBumper(0) && System.currentTimeMillis() - milliTime[0] > timeThreshold) {
        robot.drivetrain.toggleSnailDrive();
        stopwatch(0);
      }
      if (superpad.leftBumper(0) && System.currentTimeMillis() - milliTime[1] > timeThreshold) {
        robot.drivetrain.toggleHyperdrive();
        stopwatch(1);
      }
      telemetry.addData("", "Hyperdrive: " + robot.drivetrain.isHyperdriveEnabled());
      telemetry.addData("", "Snaildrive: ", robot.drivetrain.isSnailDriveEnabled());
      telemetry.update();
    }
  }

  private void stopwatch(int type) {
    milliTime[type] = System.currentTimeMillis();
  }
}
