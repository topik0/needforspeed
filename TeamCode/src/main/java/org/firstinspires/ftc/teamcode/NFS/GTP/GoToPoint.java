package org.firstinspires.ftc.teamcode.NFS.GTP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    public double yKS = 0, xKS = 0, headingKS;
    public double yClip = 1, xClip = 1, headingClip = 1;
    public boolean isDone1, isDone2, isDone3, isDone4;
    public double xError, yError, headingError;
    public SampleMecanumDrive drive;
    private HardwareMap hardwareMap;
    private static final double TAU = 2*Math.PI;
    FtcDashboard dashboard;
    VoltageSensor voltageSensor;
    public PIDFController headingPIDF, translateXPIDF, translateYPIDF;
    ElapsedTime timer;

    public GoToPoint(SampleMecanumDrive drive, HardwareMap hwMap) {
        this.drive = drive;
        isDone1 = false;
        isDone2 = false;
        isDone3 = false;
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
        isDone1 = false;
        isDone2 = false;
        isDone3 = false;
        isDone4 = false;
   }

    public void goToPointExtra(double targetX, double targetY, double targetDegrees, double targetXError, double targetYError, double targetHeadingError, double translationalErrorClip, double headingErrorClip,  double translationalClip, double hClip, double maxPower) {



        if (!isDone4) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            //kS

            if(Math.abs(headingError)<= Math.toRadians(targetHeadingError)){
                headingKS = 0;
            }
            if(Math.abs(headingError) > Math.toRadians(targetHeadingError)){
                headingKS= 0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            if(Math.abs(xError)>targetXError){
                xKS = 0;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            if(Math.abs(yError)>targetYError){
                yKS= 0;
            }


            //power clips

            if(Math.abs(headingError)<= Math.toRadians(headingErrorClip)){
                headingClip = hClip; //.5
            }
            if(Math.abs(headingError) > Math.toRadians(headingErrorClip)){
                headingClip = maxPower;
            }

            if(Math.abs(translationalErrorClip)<= 2){
                xClip = translationalClip; //.7
            }
            if(Math.abs(translationalErrorClip)> 2){
                xClip = maxPower;
            }

            if(Math.abs(translationalErrorClip)<= 2){
                yClip = translationalClip;
            }
            if(Math.abs(translationalErrorClip)> 2){
                yClip = maxPower;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -headingClip, headingClip);
            xPID = Range.clip(xPID, -xClip, xClip);
            yPID = Range.clip(yPID, -yClip, yClip);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone4 && isDone3) {
                isDone4 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone2 && !isDone3) {
                isDone3 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone1 && !isDone2) {
                isDone2 = true;
            }

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone1) {
                isDone1 = true;
            }

            if (Math.abs(xError) > targetXError || Math.abs(yError) > targetYError || Math.abs(headingError) > Math.toRadians(targetHeadingError)) {
                notDone();
            }
        }
    }


    public void goToPointPS(double targetX, double targetY, double targetDegrees, double translationalClip, double hClip) {

        double targetXError = 1;
        double targetYError = .5;
        double targetHeadingError = .75;

        if (!isDone4) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            //kS

            if(Math.abs(headingError)<= Math.toRadians(targetHeadingError)){
                headingKS = 0;
            }
            if(Math.abs(headingError) > Math.toRadians(targetHeadingError)){
                headingKS= 0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            if(Math.abs(xError)>targetXError){
                xKS = 0;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            if(Math.abs(yError)>targetYError){
                yKS= 0;
            }


            //power clips

            if(Math.abs(headingError)<= Math.toRadians(5)){
                headingClip = hClip; //.5
            }
            if(Math.abs(headingError) > Math.toRadians(5)){
                headingClip = 1;
            }

            if(Math.abs(xError)<= 2){
                xClip = translationalClip; //.7
            }
            if(Math.abs(xError)> 2){
                xClip = 1;
            }

            if(Math.abs(yError)<= 2){
                yClip = translationalClip;
            }
            if(Math.abs(yError)> 2){
                yClip = 1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -headingClip, headingClip);
            xPID = Range.clip(xPID, -xClip, xClip);
            yPID = Range.clip(yPID, -yClip, yClip);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone4 && isDone3) {
                isDone4 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone2 && !isDone3) {
                isDone3 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone1 && !isDone2) {
                isDone2 = true;
            }

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone1) {
                isDone1 = true;
            }

            if (Math.abs(xError) > targetXError || Math.abs(yError) > targetYError || Math.abs(headingError) > Math.toRadians(targetHeadingError)) {
                notDone();
            }
        }
    }


    public void goToPoint(double targetX, double targetY, double targetDegrees) {

        double targetXError = .75;
        double targetYError = .75;
        double targetHeadingError = .5;
        isDone1 = false;

        while (!isDone1) {
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
                isDone1 = true;

            }

        }
    }


    public void goToPoint(double targetX, double targetY, double targetDegrees, double targetXError, double targetYError, double targetHeadingError) {


        isDone1 = false;

        while (!isDone1) {
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
                isDone1 = true;
            }

        }

    }
    public void goToPointAuto(double targetX, double targetY, double targetDegrees) {

        double targetXError = 1;
        double targetYError = .75;
        double targetHeadingError = .75;

        if (!isDone4) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            //kS

            if(Math.abs(headingError)<= Math.toRadians(targetHeadingError)){
                headingKS = 0;
            }
            if(Math.abs(headingError) > Math.toRadians(targetHeadingError)){
                headingKS= 0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            if(Math.abs(xError)>targetXError){
                xKS = 0;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            if(Math.abs(yError)>targetYError){
                yKS= 0;
            }


            //power clips

            if(Math.abs(headingError)<= Math.toRadians(5)){
                headingClip = .5;
            }
            if(Math.abs(headingError) > Math.toRadians(5)){
                headingClip = 1;
            }

            if(Math.abs(xError)<= 2){
                xClip = .7;
            }
            if(Math.abs(xError)> 2){
                xClip = 1;
            }

            if(Math.abs(yError)<= 2){
                yClip = .7;
            }
            if(Math.abs(yError)> 2){
                yClip = 1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -headingClip, headingClip);
            xPID = Range.clip(xPID, -xClip, xClip);
            yPID = Range.clip(yPID, -yClip, yClip);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone4 && isDone3) {
                isDone4 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone2 && !isDone3) {
                isDone3 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone1 && !isDone2) {
                isDone2 = true;
            }

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone1) {
                isDone1 = true;
            }

            if (Math.abs(xError) > targetXError || Math.abs(yError) > targetYError || Math.abs(headingError) > Math.toRadians(targetHeadingError)) {
                notDone();
            }
        }
    }
    public void goToPointAuto(double targetX, double targetY, double targetDegrees, double targetXError, double targetYError, double targetHeadingError) {

        if (!isDone4) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            //kS

            if(Math.abs(headingError)<= Math.toRadians(targetHeadingError)){
                headingKS = 0;
            }
            if(Math.abs(headingError) > Math.toRadians(targetHeadingError)){
                headingKS= 0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            if(Math.abs(xError)>targetXError){
                xKS = 0;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            if(Math.abs(yError)>targetYError){
                yKS= 0;
            }


            //power clips

            if(Math.abs(headingError)<= Math.toRadians(5)){
                headingClip = .5;
            }
            if(Math.abs(headingError) > Math.toRadians(5)){
                headingClip = 1;
            }

            if(Math.abs(xError)<= 2){
                xClip = .7;
            }
            if(Math.abs(xError)> 2){
                xClip = 1;
            }

            if(Math.abs(yError)<= 2){
                yClip = .7;
            }
            if(Math.abs(yError)> 2){
                yClip = 1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -headingClip, headingClip);
            xPID = Range.clip(xPID, -xClip, xClip);
            yPID = Range.clip(yPID, -yClip, yClip);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone4 && isDone3) {
                isDone4 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone2 && !isDone3) {
                isDone3 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone1 && !isDone2) {
                isDone2 = true;
            }

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone1) {
                isDone1 = true;
            }

            if (Math.abs(xError) > targetXError || Math.abs(yError) > targetYError || Math.abs(headingError) > Math.toRadians(targetHeadingError)) {
                notDone();
            }
        }
    }


    public void goToPointSlow(double targetX, double targetY, double targetDegrees, double targetXError, double targetYError, double targetHeadingError) {


        isDone1 = false;

        while (!isDone1) {
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
                isDone1 = true;
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
        double targetHeadingError = .75;

        if (!isDone4) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            //kS

            if(Math.abs(headingError)<= Math.toRadians(targetHeadingError)){
                headingKS = 0;
            }
            if(Math.abs(headingError) > Math.toRadians(targetHeadingError)){
                   headingKS= 0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            if(Math.abs(xError)>targetXError){
                xKS = 0;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            if(Math.abs(yError)>targetYError){
                yKS= 0;
            }


            //power clips

            if(Math.abs(headingError)<= Math.toRadians(5)){
                headingClip = .5;
            }
            if(Math.abs(headingError) > Math.toRadians(5)){
                headingClip = 1;
            }

            if(Math.abs(xError)<= 2){
                xClip = .7;
            }
            if(Math.abs(xError)> 2){
                xClip = 1;
            }

            if(Math.abs(yError)<= 2){
                yClip = .7;
            }
            if(Math.abs(yError)> 2){
                yClip = 1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -headingClip, headingClip);
            xPID = Range.clip(xPID, -xClip, xClip);
            yPID = Range.clip(yPID, -yClip, yClip);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone4 && isDone3) {
                isDone4 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone2 && !isDone3) {
                isDone3 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone1 && !isDone2) {
                isDone2 = true;
            }

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone1) {
                isDone1 = true;
            }

            if (Math.abs(xError) > targetXError || Math.abs(yError) > targetYError || Math.abs(headingError) > Math.toRadians(targetHeadingError)) {
              notDone();
            }
        }
    }

    public void goToPointNonBlockingTurn(double targetX, double targetY, double targetDegrees) {

        double targetXError = 1;
        double targetYError = 1;
        double targetHeadingError = .75;

        if (!isDone4) {
            drive.update();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();

            headingError = normalizeAngleRR(Math.toRadians(targetDegrees) - (drive.getPoseEstimate().getHeading()));
            xError = targetX - drive.getPoseEstimate().getX();
            yError = targetY - drive.getPoseEstimate().getY();


            //kS

            if(Math.abs(headingError)<= Math.toRadians(targetHeadingError)){
                headingKS = 0;
            }
            if(Math.abs(headingError) > Math.toRadians(targetHeadingError)){
                headingKS= 0;
            }

            if(Math.abs(xError)<=targetXError){
                xKS = 0;
            }
            if(Math.abs(xError)>targetXError){
                xKS = 0;
            }

            if(Math.abs(yError)<=targetYError){
                yKS = 0;
            }
            if(Math.abs(yError)>targetYError){
                yKS= 0;
            }


            //power clips

            if(Math.abs(headingError)<= Math.toRadians(5)){
                headingClip = .3;
            }
            if(Math.abs(headingError) > Math.toRadians(5)){
                headingClip = .75;
            }

            if(Math.abs(xError)<= 2){
                xClip = .7;
            }
            if(Math.abs(xError)> 2){
                xClip = 1;
            }

            if(Math.abs(yError)<= 2){
                yClip = .7;
            }
            if(Math.abs(yError)> 2){
                yClip = 1;
            }


            double headingPID =  headingPIDF.calculate(0, headingError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double xPID = translateXPIDF.calculate(0, xError) + voltageCompensation * 12 / voltageSensor.getVoltage();
            double yPID =   translateYPIDF.calculate(0, yError) + voltageCompensation * 12 / voltageSensor.getVoltage();

            headingPID = Range.clip(headingPID, -headingClip, headingClip);
            xPID = Range.clip(xPID, -xClip, xClip);
            yPID = Range.clip(yPID, -yClip, yClip);

            Vector2d fieldCentric = new Vector2d(xPID, yPID).rotated(-drive.getPoseEstimate().getHeading());
            drive.setWeightedDrivePower(new Pose2d(fieldCentric.getX() + Math.copySign(xKS, xPID), fieldCentric.getY() + Math.copySign(yKS, yPID), headingPID + Math.copySign(headingKS, headingPID)));
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone4 && isDone3) {
                isDone4 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone2 && !isDone3) {
                isDone3 = true;
            }
            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && isDone1 && !isDone2) {
                isDone2 = true;
            }

            if (Math.abs(xError) < targetXError && Math.abs(yError) < targetYError && Math.abs(headingError) < Math.toRadians(targetHeadingError) && !isDone1) {
                isDone1 = true;
            }

            if (Math.abs(xError) > targetXError || Math.abs(yError) > targetYError || Math.abs(headingError) > Math.toRadians(targetHeadingError)) {
                notDone();
            }
        }
    }

    public void goToPointTele(double targetX, double targetY, double targetDegrees) {

        double targetXError = 1;
        double targetYError = .75;
        double targetHeadingError = .5;
        isDone1 = false;

        while (!isDone1) {
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
                isDone1 = true;
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