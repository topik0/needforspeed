package org.firstinspires.ftc.teamcode.NFS.GTP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;
@Config

public class GoToPoint {
    public static double voltageCompensation = 0;
    public static double headingP = 1.5;
    public static double headingD = 0.1;
    public static  double xP = .1;
    public static  double yP = .1;
    public static double xD = 0;
    public static  double yD = .0;
    public double yKS = 0, xKS = 0, headingKS = 0;
    public boolean isDone;
    public double xError, yError, headingError;
    public SampleMecanumDrive drive;
    private HardwareMap hardwareMap;
    private static final double TAU = 2*Math.PI;
    FtcDashboard dashboard;
    VoltageSensor voltageSensor;
    public PIDFController headingPIDF, translateXPIDF, translateYPIDF;

    public GoToPoint(SampleMecanumDrive drive, HardwareMap hwMap) {
        this.drive = drive;
        isDone = false;
        dashboard = FtcDashboard.getInstance();
        hardwareMap = hwMap;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        headingPIDF = new PIDFController(headingP, 0, headingD, 0);
        translateXPIDF = new PIDFController(xP, 0, xD, 0);
        translateYPIDF = new PIDFController(yP, 0, yD, 0);
    }

    public void reset() {
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }
    public void setPos(double x, double y, double heading){
        drive.setPoseEstimate(new Pose2d(x, y, heading));
    }
    public void notDone() {
        isDone = false;
   }






    public void goToPoint(double targetX, double targetY, double targetDegrees) {

        double targetXError = .75;
        double targetYError = .75;
        double targetHeadingError = .5;
        isDone = false;

        while (!isDone) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            if(headingError<= targetHeadingError){
                headingKS = 0;
            }
            else if(headingError> targetHeadingError){
            //    headingKS=.1;
            }

            if(xError<=targetXError){
                xKS = 0;
            }
            else if(xError>targetXError){
             //   xKS=.1;
            }

            if(yError<=targetYError){
                yKS = 0;
            }
            else if(yError>targetYError){
             //   yKS=.1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -1, 1);
            xPID = Range.clip(xPID, -1, 1);
            yPID = Range.clip(yPID, -1, 1);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError)) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                isDone = true;

            }

        }
    }


    public void goToPoint(double targetX, double targetY, double targetDegrees, double targetXError, double targetYError, double targetHeadingError) {


        isDone = false;

        while (!isDone) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            if(headingError<= targetHeadingError){
                headingKS = 0;
            }
            else if(headingError> targetHeadingError){
                headingKS=.1;
            }

            if(xError<=targetXError){
                xKS = 0;
            }
            else if(xError>targetXError){
                xKS=.1;
            }

            if(yError<=targetYError){
                yKS = 0;
            }
            else if(yError>targetYError){
                yKS=.1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -1, 1);
            xPID = Range.clip(xPID, -1, 1);
            yPID = Range.clip(yPID, -1, 1);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError)) {
                isDone = true;
            }

        }

    }



    public void goToPointSlow(double targetX, double targetY, double targetDegrees, double targetXError, double targetYError, double targetHeadingError) {


        isDone = false;

        while (!isDone) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            if(headingError<= targetHeadingError){
                headingKS = 0;
            }
            else if(headingError> targetHeadingError){
                headingKS=.05;
            }

            if(xError<=targetXError){
                xKS = 0;
            }
            else if(xError>targetXError){
                xKS=0;
            }

            if(yError<=targetYError){
                yKS = 0;
            }
            else if(yError>targetYError){
                yKS=0;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -.7, .7);
            xPID = Range.clip(xPID, -.3, .3);
            yPID = Range.clip(yPID, -.3, .3);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError)) {
                isDone = true;
            }

        }

    }

    //turns to a relative angle
    public void turn(double targetDegrees) {
        goToPoint(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + targetDegrees);
    }


    //turns to an absolute angle (untested)
    public void turnTo(double targetDegrees) {
        goToPoint(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), targetDegrees);
    }

    public void goToPointNonBlocking(double targetX, double targetY, double targetDegrees) {

        double targetXError = 1;
        double targetYError = .75;
        double targetHeadingError = .5;
        isDone = false;

        if (!isDone) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            if(Math.abs(headingError)<= targetHeadingError){
                headingKS = 0;
            }
            else if(Math.abs(headingError) > targetHeadingError){
                   headingKS=0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            else if(Math.abs(xError)>targetXError){
                 // xKS=.1;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            else if(Math.abs(yError)>targetYError){
                 //  yKS=.1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -1, 1);
            xPID = Range.clip(xPID, -1, 1);
            yPID = Range.clip(yPID, -1, 1);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError)) {
                isDone = true;
            }

        }
    }

    public void goToPointTele(double targetX, double targetY, double targetDegrees) {

        double targetXError = 1;
        double targetYError = .75;
        double targetHeadingError = .5;
        isDone = false;

        while (!isDone) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            if(Math.abs(headingError)<= targetHeadingError){
                headingKS = 0;
            }
            else if(Math.abs(headingError) > targetHeadingError){
                headingKS=0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            else if(Math.abs(xError)>targetXError){
                // xKS=.1;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            else if(Math.abs(yError)>targetYError){
                //  yKS=.1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -1, 1);
            xPID = Range.clip(xPID, -1, 1);
            yPID = Range.clip(yPID, -1, 1);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError)) {
                isDone = true;
            }

        }
    }

    public void turnTele(double targetDegrees) {
        goToPointTele(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + targetDegrees);
    }

    public double normalizeAngleRR(double angle) {
        while (angle > Math.PI)
            angle -= TAU;
        while (angle < -Math.PI)
            angle += TAU;
        return angle;
    }
    }