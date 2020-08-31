package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
  /*
  Robot constructor
   */
  protected HardwareMap hwMap;
  public Drivetrain drivetrain;

  public Robot(HardwareMap hwMap) {
    this.hwMap = hwMap;
    drivetrain = new Drivetrain(hwMap);
    initHardware();
  }
  /*
  Initializes the robot and the components
   */
  public void initHardware() {}

  public void startPositions() {}

  private void sleep(int milliseconds) {
    sleep(milliseconds);
  }
}
