package org.firstinspires.ftc.teamcode.Team9113;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Team9113.Control.Omnipad;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;

@Config
@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    private static double offSetAngle = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Omnipad pad = new Omnipad(gamepad1, gamepad2, robot);
        // Set things to starting positions
        robot.startPositions();
        // Initialize variables
        FtcDashboard dashboard = FtcDashboard.getInstance();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            robot.flywheels.run();
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double ly = pad.getLeftY();
            double lx = pad.getLeftX();
            double rx = pad.getRightX();
            double heading = angles.firstAngle - offSetAngle + 90;
            double speed = Math.hypot(ly, lx);
            double y = pad.getY(speed, heading, ly, lx);
            double x = pad.getX(speed, heading, ly, lx);
            robot.drivetrain.driveFieldCentric(x, y, rx);
            if (pad.drivetrainDormant()) robot.drivetrain.brake();
            if (pad.shootRing()) robot.shootDisc();
            if (pad.flywheelsToggle()) robot.flywheels.togglePID();
            if (pad.setOffset()) offSetAngle = angles.firstAngle;
            // todo: turn for powershots (6 deg)
            if (pad.raiseFlap()) robot.flapUpperPosition();
            if (pad.lowerFlap()) robot.flapLowerPosition();
            if (pad.intakeToggle()) robot.toggleIntake();
            if (pad.clawToggle()) robot.toggleClaw();
            if (pad.wobbleToggle()) robot.toggleWobble();
            if (pad.intakeReverse()) robot.reverseIntake();
            else if (robot.intakeRunning) robot.startIntake();
            telemetry.addData("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}