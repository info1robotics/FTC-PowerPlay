package org.firstinspires.ftc.teamcode.SubSystems.V3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class Lift {
    public static int LOW_POS = 350;
    public static int MID_POS = 1050;
    public static int HIGH_POS = 1850;
    public static int COLLECT_POS = 0;
    public DcMotorEx liftLeft, liftRight;
    public MotorConfigurationType mct1, mct2;

    public Lift(LinearOpMode opMode) {
        liftLeft = opMode.hardwareMap.get(DcMotorEx.class, "LiftLeft");
        liftRight = opMode.hardwareMap.get(DcMotorEx.class, "LiftRight");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mct1 = liftLeft.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        liftLeft.setMotorType(mct1);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mct2 = liftLeft.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        liftLeft.setMotorType(mct2);
    }

    public void setHeight(int targetHeight, double power) {
        setTargetPosition(targetHeight);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        liftLeft.setMode(runMode);
        liftRight.setMode(runMode);
    }

    public void resetEncoders() {
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power) {
        liftRight.setPower(power);
        liftLeft.setPower(power);
    }

    public void setTargetPosition(int ticks) {
        liftRight.setTargetPosition(ticks);
        liftLeft.setTargetPosition(ticks);
    }


    public void debug() {
    }
}
