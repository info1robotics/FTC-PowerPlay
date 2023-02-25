package org.firstinspires.ftc.teamcode.SubSystems.V2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Linkage{
    public DcMotorEx linkageLeft, linkageRight;
    public int DEFAULT_HEIGHT  =  -10;
    public int GROUND_JUNCTION =  100;
    public int LOW_JUNCTION    =  250;
    public int MID_JUNCTION    =  400;
    public int HIGH_JUNCTION   =  550;
    public int THRESHOLD       =    8;
    public int LOWER_LIMIT     = -100;
    public int HIGHER_LIMIT    =  725;
    public int TARGET_HEIGHT   =    0;

    public Linkage(LinearOpMode opMode){
    linkageLeft = opMode.hardwareMap.get(DcMotorEx.class, "linkageLeft");
    linkageRight = opMode.hardwareMap.get(DcMotorEx.class, "linkageRight");

    linkageRight.setDirection(DcMotorSimple.Direction.REVERSE);
    linkageLeft.setDirection(DcMotorSimple.Direction.FORWARD);

    linkageRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    linkageLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeight(int targetHeight, double power){
        if(targetHeight > HIGHER_LIMIT) targetHeight = HIGHER_LIMIT;
        if(targetHeight < LOWER_LIMIT) targetHeight = LOWER_LIMIT;

        setTargetPosition(targetHeight);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        linkageLeft.setMode(runMode);
        linkageRight.setMode(runMode);
    }

    public void resetEncoders(){
        linkageRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

     public void setPower(double power){
        linkageRight.setPower(power);
        linkageLeft.setPower(power);
    }

    public void setTargetPosition(int ticks){
        linkageRight.setTargetPosition(ticks);
        linkageLeft.setTargetPosition(ticks);
    }

    public boolean hasReachedTarget(){
        return Math.abs(linkageLeft.getCurrentPosition() - linkageLeft.getTargetPosition()) < 10 || Math.abs(linkageRight.getCurrentPosition() - linkageRight.getTargetPosition()) < 10;
    }

    public void debug() {
        telemetry.addData("Left Motor Tick Count ", linkageLeft.getCurrentPosition());
        telemetry.addData("Right Motor Tick Count ", linkageRight.getCurrentPosition());
    }
}
