package org.firstinspires.ftc.teamcode.Team9113.Control;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Superpad {
  private Gamepad[] gamepad = new Gamepad[4];

  public Superpad(Gamepad gamepadOne, Gamepad gamepadTwo) {
    gamepad[0] = gamepadOne;
    gamepad[1] = gamepadTwo;
  }

  public double leftJoystickY(int pad) {
    return gamepad[pad].left_stick_y;
  }

  public double leftJoystickX(int pad) {
    return gamepad[pad].left_stick_x;
  }

  public double rightJoystickY(int pad) {
    return gamepad[pad].right_stick_y;
  }

  public double rightJoystickX(int pad) {
    return gamepad[pad].right_stick_x;
  }

  public boolean dpadUp(int pad) {
    return gamepad[pad].dpad_up;
  }

  public boolean dpadDown(int pad) {
    return gamepad[pad].dpad_down;
  }

  public boolean dpadLeft(int pad) {
    return gamepad[pad].dpad_left;
  }

  public boolean dpadRight(int pad) {
    return gamepad[pad].dpad_right;
  }

  public boolean leftBumper(int pad) {
    return gamepad[pad].left_bumper;
  }

  public boolean rightBumper(int pad) {
    return gamepad[pad].right_bumper;
  }

  public double leftTrigger(int pad) {
    return gamepad[pad].left_trigger;
  }

  public double rightTrigger(int pad) {
    return gamepad[pad].right_trigger;
  }

  public boolean a(int pad) {
    return gamepad[pad].a;
  }

  public boolean b(int pad) {
    return gamepad[pad].b;
  }

  public boolean x(int pad) {
    return gamepad[pad].x;
  }

  public boolean y(int pad) {
    return gamepad[pad].y;
  }

  public boolean back(int pad) {
    return gamepad[pad].back;
  }

  public boolean guide(int pad) {
    return gamepad[pad].guide;
  }

  public boolean start(int pad) {
    return gamepad[pad].start;
  }


}
