package org.firstinspires.ftc.teamcode.Team9113;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain extends Robot{
    /**
     * upperLeft [0]  upperRight [1]
     * lowerLeft [2] lowerRight [3]
     */
    private double currentThrottle;
    private double currentStrafeThrottle;
    private final double setHyperdriveThrottle = 1.0;
    private final double setStrafeHyperdriveThrottle = 1.0;
    private final double setBirdThrottle = .7;
    private final double setStrafeBirdThrottle = .7;
    private final double setSnailStrafeThrottle = .37;
    private final double setSnailThrottle = .37;

    private DcMotor[] drivetrain = new DcMotor[4];
    private final String[] drivetrainNames = {"upperLeft", "upperRight", "lowerLeft", "lowerRight"};

    public Drivetrain(){
        for(int i = 0; i < 4; i++)
            drivetrain[i] = hardwareMap.dcMotor.get(drivetrainNames[i]);
    }

    public void toggleHyperdrive(){
        
    }

    public void enableHyperdrive(){
        currentThrottle = setHyperdriveThrottle;
        currentStrafeThrottle = setStrafeHyperdriveThrottle;
    }

    public void disableHyperdrive(){
        currentThrottle = setSnailThrottle;
        currentStrafeThrottle = setSnailStrafeThrottle;
    }

    public boolean isSnailDriveEnabled(){
        return currentThrottle == setSnailThrottle && currentStrafeThrottle == setSnailStrafeThrottle;
    }

    public boolean isHyperdriveEnabled(){
        return currentThrottle == setHyperdriveThrottle && strafeSpeed == setStrafeHyperdriveThrottle;
    }

    public void strafeLeft(int ticks, double speed){
        if(ticks == 0)
            strafeLeftWithoutEncoders();
        else strafeLeftWithEncoders(ticks, speed);
    }

    private void strafeLeftWithEncoders(int ticks, double speed){
        stopAndResetAllMotorEncoders();

        for(int i = 0; i <= 3; i+=3)
            drivetrain[i].setTargetPosition(-ticks);
        for(int i = 1; i <= 2; i++)
            drivetrain[i].setTargetPosition(ticks);

        setAllMotorsRunToPosition();

        for(int i = 0; i <= 3; i+=3)
            drivetrain[i].setPower(-currentStrafeThrottle);
        for(int i = 1; i <= 2; i++)
            drivetrain[i].setPower(currentStrafeThrottle);

        while(drivetrain[0].isBusy()) {
            telemetry.addData("", "Runtime: ", drivetrain[0].getCurrentPosition());
            telemetry.update();
        }
    }

    private void strafeLeftWithoutEncoders(){
        for(int i = 0; i <= 3; i+=3)
            drivetrain[i].setPower(-currentStrafeThrottle);
        for(int i = 1; i <= 2; i++)
            drivetrain[i].setPower(currentStrafeThrottle);
    }

    private void strafeRightWithoutEncoders(){
        for(int i = 0; i <= 3; i+=3)
            drivetrain[i].setPower(currentStrafeThrottle);
        for(int i = 1; i <= 2; i++)
            drivetrain[i].setPower(-currentStrafeThrottle);
    }

    public void moveForward(){
        setPower(currentStrafeThrottle);
    }

    public void moveBackward(){
        setPower(-currentStrafeThrottle);
    }

    public void moveLive(double speed){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setPower(currentThrottle*speed);
    }

    public void moveLeftMotorsLive(double speed){
        for(int i = 0; i <= 2; i+=2)
            drivetrain[i].setPower(currentThrottle*speed);
    }

    public void moveRightMotorsLive(double speed){
        for(int i = 1; i <= 3; i+=2)
            drivetrain[i].setPower(currentThrottle*speed);
    }

    public void brakeMotors(){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turn180(char direction){
        if(direction == 'r')
            turnRight(2800, .8);
        else if(direction == 'l')
            turnLeft(2800, .8);
        brakeMotors();
    }

    public void turn90(char direction){
        if(direction == 'r')
            turnRight(1400, 1);
        else if(direction == 'l')
            turnLeft(1400, 1);
        brakeMotors();
    }

    private void setAllMotorsRunToPosition(int ticks){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setTargetPosition(ticks);
    }

    private void stopAndResetAllMotorEncoders(){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setAllMotorsRunToPosition(){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setAllMotorsRunWithoutEncoders(){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setAllMotorsRunUsingEncoders(){
        for(int i = 0; i < 4; i++)
            drivetrain[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power){
        setAllMotorsRunWithoutEncoders();
        for(int i = 0; i < 4; i++)
            drivetrain[i].setPower(power);
    }

    public void setLeftPower(double power){
        setAllMotorsRunWithoutEncoders();
        for(int i = 0; i <= 2; i+=2)
            drivetrain[i].setPower(power);
    }

    public void setRightPower(double power){
        setAllMotorsRunWithoutEncoders();
        for(int i = 1; i <= 3; i+=2)
            drivetrain[i].setPower(power);
    }

    public void zeroPowerMotors(){
        setAllMotorsRunWithoutEncoders();
        for(int i = 0; i < 4; i++)
            drivetrain[i].setPower(0);
    }

    public double getThrottle(){ return currentThrottle; }
    public double getStrafeThrottle(){ return currentStrafeThrottle; }
}
