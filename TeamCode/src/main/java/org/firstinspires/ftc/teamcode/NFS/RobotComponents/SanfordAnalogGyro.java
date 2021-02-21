package org.firstinspires.ftc.teamcode.NFS.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NFS.GTP.GoToPoint;
import org.firstinspires.ftc.teamcode.NFS.drive.SampleMecanumDrive;
@Config
public class SanfordAnalogGyro {
    HardwareMap hardwareMap;
    GoToPoint point;
    SampleMecanumDrive drive;
    AnalogInput sanGyro;
    public static double correctionCaff = 0.98360655737;
    public double maxV = 0;
    boolean firstUpdateLoop;
    boolean reset;
    double prevAngle;
    double cumulativeAngle;
    public SanfordAnalogGyro(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        sanGyro = hardwareMap.get(AnalogInput.class, "SanfordAnalogGyro");
        drive = new SampleMecanumDrive(hardwareMap);
        point = new GoToPoint(drive, hardwareMap);
        firstUpdateLoop = true;
        reset = false;
        cumulativeAngle = 0;

    }
    public double rawVoltage(){
        return sanGyro.getVoltage();
    }
    public double getRawAngle(){
        return (sanGyro.getVoltage() / 3.3) * 2 * Math.PI;
    }
    public double testAngle(){
        double angle;
        if(maxV > 0){
            angle = rawVoltage() / maxV * 360;
        }
        else angle = 0;
        return angle;
    }
    public void update(){
        if(firstUpdateLoop){
            prevAngle = getRawAngle();
            firstUpdateLoop = false;
        }
        if(rawVoltage()>maxV){
            maxV=rawVoltage();
        }
        double currentAngle = getRawAngle();
        cumulativeAngle += point.normalizeAngleRR(currentAngle-prevAngle) * correctionCaff;
        prevAngle = currentAngle;
    }
    public double getCorrectedAngle() {
        return point.normalizeAngleRR(cumulativeAngle);
    }
}
