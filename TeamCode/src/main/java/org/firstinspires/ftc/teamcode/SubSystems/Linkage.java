package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Linkage {

    public DcMotor linkageLeft;
    public DcMotor linkageRight;

    public static int CURRENT_LEVEL = 0;

    public static final int GROUND_LEVEL =      10;
    public static final int JUNCTION_LEVEL =    20;
    public static final int LOW_LEVEL =         50;
    public static final int MID_LEVEL =         200;
    public static final int HIGH_LEVEL =        250;

    public static final int LINKAGE_THRESHOLD = 10;
    public static final int SAFETY_THRESHOLD = 100;

    private static final int LINKAGE_MIN = 10;
    private static final int LINKAGE_MAX = 500;

    public Linkage(LinearOpMode opMode){
        linkageLeft = opMode.hardwareMap.get(DcMotor.class, "linkageLeft");
        linkageRight = opMode.hardwareMap.get(DcMotor.class, "linkageRight");

        linkageLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        linkageRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linkageLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linkageRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RESET_ENCODERS(){
        linkageLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void SET_MOTORS_RUNMODE(){
        linkageLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linkageRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SET_TARGET_POSITION(int LEVEL){
        linkageRight.setTargetPosition(LEVEL);
        linkageLeft.setTargetPosition(LEVEL);
    }

    public void DEBUG(){
        telemetry.addData("Left Linkage Tick Count ", linkageLeft.getCurrentPosition());
        telemetry.addData("Right Linkage Tick Count ", linkageRight.getCurrentPosition());
    }

    public void SET_MOTOR_POWER(double pw){
        linkageRight.setPower(pw);
        linkageLeft.setPower(pw);
    }

    public void GO_TO_LEVEL(int LEVEL, double SPEED){
        if(LEVEL > LINKAGE_MAX) LEVEL = LINKAGE_MAX;
        if(LEVEL < LINKAGE_MIN) LEVEL = LINKAGE_MIN;
        SET_TARGET_POSITION(LEVEL);
        SET_MOTORS_RUNMODE();
        SET_MOTOR_POWER(SPEED);
    }
}
