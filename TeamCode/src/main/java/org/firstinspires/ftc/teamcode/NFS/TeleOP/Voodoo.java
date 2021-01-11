package org.firstinspires.ftc.teamcode.NFS.TeleOP;

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
import org.firstinspires.ftc.teamcode.NFS.Control.Omnipad;
import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.NFS.RobotComponents.Robot;

/**
 * @author Topik
 * @version 1.0
 * @since 1.0
 * This class is the main TeleOP OP Mode, which contains all of the robot controls
 */
@Config
@TeleOp(name = "Voodoo", group = "Linear Opmode")
public class Voodoo extends LinearOpMode {
    /**
     * The angles for the constant turns
     */
    public static double turnRightAngle = -6, turnLeftAngle = 6;
    public static double flywheelTimerThreshold = 200, armTimerThreshold = 200;
    /**
     * Controls whether or not active breaking is enabled
     */
    public static boolean activeBreakingEnabled = true;
    private static double offSetAngle = 0;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        Omnipad pad = new Omnipad(gamepad1, gamepad2, robot);
        StopWatch flywheelStopwatch = new StopWatch();
        StopWatch armStopwatch = new StopWatch();
        robot.startPositions();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        BNO055IMU imu = robot.genesis.imu;
        imu.initialize(parameters);
        waitForStart();
        while (opModeIsActive()) {
            robot.flywheels.run();
            robot.flicker.checkState();
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double ly = pad.getLeftY();
            double lx = pad.getLeftX();
            double rx = pad.getRightX();
            rx = Math.sqrt(Math.abs(rx)) * rx * Drivetrain.turnThrottle;
            double heading = angles.firstAngle - offSetAngle + 90;
            double speed = Math.hypot(ly, lx);
            double y = pad.getY(speed, heading, ly, lx);
            double x = pad.getX(speed, heading, ly, lx);
            robot.drivetrain.driveFieldCentric(x, y, rx);
            if (activeBreakingEnabled && pad.drivetrainDormant()) robot.drivetrain.brake();
            if (pad.shootRing()) robot.flicker.shootOut();
            if (pad.flywheelsToggle()) robot.flywheels.togglePID();
            if (pad.setOffset()) offSetAngle = angles.firstAngle;
            if (pad.raiseFlap()) robot.flap.goToHighGoalPosition();
            else if (pad.lowerFlap()) robot.flap.goToPowershotPosition();
            if (pad.intakeToggle()) robot.intake.toggle();
            if (pad.clawToggle()) robot.claw.toggle();
            if (pad.armToggle()) robot.arm.toggle();
            if (pad.intakeReverse()) robot.intake.reverse();
            if (pad.turnRight()) robot.drivetrain.turn(turnRightAngle);
            else if (pad.turnLeft()) robot.drivetrain.turn(turnLeftAngle);
            else if (robot.intake.isRunning()) robot.intake.start();
            if (robot.flywheels.running() && flywheelStopwatch.getTime() >= flywheelTimerThreshold)
                robot.drivetrain.setTurnThrottle(.5);
            else if (!robot.arm.isUp() && armStopwatch.getTime() >= armTimerThreshold)
                robot.drivetrain.setTurnThrottle(.6);
            else robot.drivetrain.setTurnThrottle(.75);
            telemetry.addData("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            telemetry.addData("Flicker State", robot.flicker.flickerState());
            telemetry.addData("Flywheel Runstate", robot.flywheels.getRunState());
            telemetry.addData("Arm State", robot.arm.getState());
            telemetry.addData("Claw State", robot.claw.getState());
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}