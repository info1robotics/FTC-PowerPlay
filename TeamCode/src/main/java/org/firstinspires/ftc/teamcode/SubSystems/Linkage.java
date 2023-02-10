package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Linkage {
    public static final int GROUND_LEVEL = -10;
    public static final int JUNCTION_LEVEL = 100;
    public static final int LOW_LEVEL = 250;
    public static final int MID_LEVEL = 400;
    public static final int HIGH_LEVEL = 550;
    public static final int LINKAGE_THRESHOLD = 8;
    public static final int SAFETY_THRESHOLD = 50;
    public static final int LINKAGE_MIN = -25;
    public static final int LINKAGE_MAX = 725;
    public static int CURRENT_LEVEL = 0;
    public static int DESIRED_HEIGHT = 0;
    public DcMotor linkageLeft;
    public DcMotor linkageRight;

    public Linkage(LinearOpMode opMode) {
        linkageLeft = opMode.hardwareMap.get(DcMotor.class, "LinkageLeft");
        linkageRight = opMode.hardwareMap.get(DcMotor.class, "LinkageRight");

        linkageLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        linkageRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linkageLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linkageRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linkageLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry = opMode.telemetry;
    }

    public void resetEncoders() {
        linkageLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        linkageLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linkageRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(int LEVEL) {
        linkageRight.setTargetPosition(LEVEL);
        linkageLeft.setTargetPosition(LEVEL);
    }

    public void debug() {
        telemetry.addData("Left Linkage Tick Count ", linkageLeft.getCurrentPosition());
        telemetry.addData("Right Linkage Tick Count ", linkageRight.getCurrentPosition());
    }

    public void setPower(double pw) {
        linkageRight.setPower(pw);
        linkageLeft.setPower(pw);
    }

    public void goToLevel(int LEVEL, double SPEED) {
        if (LEVEL > LINKAGE_MAX) LEVEL = LINKAGE_MAX;
        if (LEVEL < LINKAGE_MIN) LEVEL = LINKAGE_MIN;
        setTargetPosition(LEVEL);
        runToPosition();
        setPower(SPEED);
    }
    public void goToLevelAuto(int LEVEL, double SPEED) {
        if (LEVEL > LINKAGE_MAX) LEVEL = LINKAGE_MAX;
        if (LEVEL < LINKAGE_MIN) LEVEL = LINKAGE_MIN;
        if(DESIRED_HEIGHT > 550) SPEED = 0.15;
        setTargetPosition(LEVEL);
        runToPosition();
        setPower(SPEED);
    }


    public void update(){
        goToLevelAuto(DESIRED_HEIGHT, 0.3);
    }
}
