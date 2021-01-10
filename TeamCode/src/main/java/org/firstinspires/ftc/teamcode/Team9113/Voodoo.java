package org.firstinspires.ftc.teamcode.Team9113;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.lang3.time.StopWatch;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Team9113.Control.Omnipad;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Team9113.Robot.Robot;

@Config
@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    private static double offSetAngle = 0;
    public static double turnRightAngle = 6, turnLeftAngle = -6;
    public static double flywheelTimerThreshold = 200, armTimerThreshold = 200;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Omnipad pad = new Omnipad(gamepad1, gamepad2);
        StopWatch flywheelStopwatch = new StopWatch();
        StopWatch armStopwatch = new StopWatch();
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
            robot.flicker.checkState();
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double ly = pad.getLeftY();
            double lx = pad.getLeftX();
            double rx = pad.getRightX();
            rx = -1 * Math.sqrt(Math.abs(rx)) * rx * Drivetrain.turnThrottle;
            double heading = angles.firstAngle - offSetAngle + 90;
            double speed = Math.hypot(ly, lx);
            double y = pad.getY(speed, heading, ly, lx);
            double x = pad.getX(speed, heading, ly, lx);
            robot.drivetrain.driveFieldCentric(x, y, rx);
            if (pad.drivetrainDormant()) robot.drivetrain.brake();
            if (pad.shootRing()) robot.flicker.shootOut();
            if (pad.flywheelsToggle()) {
                robot.flywheels.togglePID();
            }
            if (pad.setOffset()) offSetAngle = angles.firstAngle;
            if (pad.raiseFlap()) robot.flap.goToHighGoalPosition();
            if (pad.lowerFlap()) robot.flap.goToPowershotPosition();
            if (pad.intakeToggle()) robot.intake.toggle();
            if (pad.clawToggle() && !robot.arm.isUp()) robot.claw.toggle();
            if (pad.armToggle()) robot.arm.toggle();
            if (pad.intakeReverse()) robot.intake.reverse();
            if (pad.turnRight()) robot.drivetrain.mecanumDrive.turn(Math.toRadians(turnRightAngle));
            else if (pad.turnLeft())
                robot.drivetrain.mecanumDrive.turn(Math.toRadians(turnLeftAngle));
            else if (robot.intake.isRunning()) robot.intake.start();
            if (robot.flywheels.running() && flywheelStopwatch.getTime() >= flywheelTimerThreshold)
                Drivetrain.setTurnThrottle(.5);
            else if (!robot.arm.isUp() && armStopwatch.getTime() >= armTimerThreshold)
                Drivetrain.setTurnThrottle(.6);
            else Drivetrain.setTurnThrottle(.75);
            telemetry.addData("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}