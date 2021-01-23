package org.firstinspires.ftc.teamcode.NFS.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    /**
     * The timer values for different drivetrain throttles depending on the flywheel and arm states
     */
    public static double flywheelTimerThreshold = 400, armTimerThreshold = 300;
    /**
     * The turn throttle values to be set in certain cases depending on robot component states
     */
    public static double flywheelsTurnThrottle = .5, armTurnThrottle = .6, normalTurnThrottle = .6;
    /**
     * The drivetrain throttle values to be set in certain cases depending on robot component states
     */
    public static double flywheelsDrivetrainThrottle = .75, armDrivetrainThrottle = 1, normalDrivetrainThrottle = 1;
    /**
     * Controls whether or not active breaking is enabled
     */
    public static boolean activeBreakingEnabled = true;
    public static boolean highGoalAngle = true;
    private static double offSetAngle = 0;
    public ElapsedTime flywheelTimer = new ElapsedTime();
    public ElapsedTime armTimer = new ElapsedTime();

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
            //rx = Math.sqrt(Math.abs(rx)) * rx * robot.drivetrain.getTurnThrottle();
            double heading = angles.firstAngle - offSetAngle + Math.toRadians(270);
            double speed = Math.hypot(ly, lx);
            double y = pad.getY(speed, heading, ly, lx);
            double x = pad.getX(speed, heading, ly, lx);
            robot.drivetrain.driveFieldCentric(x, y, rx);
            if (activeBreakingEnabled && pad.drivetrainDormant()) robot.drivetrain.brake();
            if (pad.shootRing()) robot.flicker.shootOut();
            if (pad.flywheelsToggle()) {
                robot.flywheels.togglePID();
                flywheelTimer.reset();
                /*try {
                    if (robot.flywheels.running()) {
                        flywheelStopwatch.start();
                    } else flywheelStopwatch.reset();
                } catch (Exception ignored) {
                } */
            }
            if (pad.setOffset()) offSetAngle = angles.firstAngle;
            if (pad.raiseFlap()) {
                robot.flap.goToHighGoalPosition();
                highGoalAngle = true;
            } else if (pad.lowerFlap()) {
                robot.flap.goToPowershotPosition();
                highGoalAngle = false;
            }
            if (pad.intakeToggle()) robot.intake.toggle();
            if (pad.intakeReverse()) robot.intake.reverse();
            else if (robot.intake.isRunning()) robot.intake.start();
            if (pad.clawToggle()) robot.claw.toggle();
            if (pad.armToggle()) {
                robot.arm.toggle();
                armTimer.reset();
                try {
                    if (!robot.arm.isUp()) {
                        robot.flap.goFlush();
                        armStopwatch.start();
                    } else armStopwatch.reset();
                } catch (Exception ignored) {
                }
            }

            //lowering flap when wobble is out and putting it back up after
            if (!robot.arm.isUp()) {
                robot.flap.goFlush();
            } else if (robot.arm.isUp() && highGoalAngle && armTimer.milliseconds() >= 500) {
                robot.flap.goToHighGoalPosition();
            } else if (robot.arm.isUp() && !highGoalAngle && armTimer.milliseconds() >= 500) {
                robot.flap.goToPowershotPosition();
            }

            //powershot imu turns
            if (pad.turnRight()) robot.drivetrain.turn(turnRightAngle);
            else if (pad.turnLeft()) robot.drivetrain.turn(turnLeftAngle);

            //turn and drivetrain throttles
            if (robot.flywheels.running() && flywheelTimer.milliseconds() >= flywheelTimerThreshold && robot.arm.isUp()) {
                robot.drivetrain.setTurnThrottle(flywheelsTurnThrottle);
                robot.drivetrain.setDrivetrainThrottle(flywheelsDrivetrainThrottle);
                //flywheelStopwatch.reset();
            } else if (!robot.arm.isUp() && armTimer.milliseconds() >= armTimerThreshold) {
                robot.drivetrain.setTurnThrottle(armTurnThrottle);
                robot.drivetrain.setDrivetrainThrottle(armDrivetrainThrottle);
                //armStopwatch.reset();
            } else {
                robot.drivetrain.setTurnThrottle(normalTurnThrottle);
                robot.drivetrain.setDrivetrainThrottle(normalDrivetrainThrottle);
            }

            //phone telemetry
            telemetry.addData("Flicker State", robot.flicker.getState());
            telemetry.addData("Flywheel Velocity", Math.abs(robot.flywheels.flywheelFront.getCorrectedVelocity()));
            telemetry.addData("Flywheel Runstate", robot.flywheels.getRunState());
            telemetry.addData("Flywheel Velostate", robot.flywheels.getVelocityState());
            telemetry.addData("Arm State", robot.arm.getState());
            telemetry.addData("Claw State", robot.claw.getState());
            telemetry.addData("rx", rx);
            telemetry.addData("turn throttle", robot.drivetrain.getTurnThrottle());
            telemetry.addData("dt throttle", robot.drivetrain.getThrottle());
            telemetry.update();

            //dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Flywheel Velocity", robot.flywheels.flywheelFront.getCorrectedVelocity());
            packet.put("Flywheel Target Velocity", robot.flywheels.getTargetVelocity());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
