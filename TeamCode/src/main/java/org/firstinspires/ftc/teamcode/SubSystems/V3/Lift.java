package org.firstinspires.ftc.teamcode.SubSystems.V3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Lift{
    public DcMotorEx liftLeft, liftRight;

    public Lift(LinearOpMode opMode){
        liftLeft = opMode.hardwareMap.get(DcMotorEx.class, "LiftLeft");
        liftRight = opMode.hardwareMap.get(DcMotorEx.class, "LiftRight");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeight(int targetHeight, double power){
        setTargetPosition(targetHeight);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        liftLeft.setMode(runMode);
        liftRight.setMode(runMode);
    }

    public void resetEncoders(){
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power){
        liftRight.setPower(power);
        liftLeft.setPower(power);
    }

    public void setTargetPosition(int ticks){
        liftRight.setTargetPosition(ticks);
        liftLeft.setTargetPosition(ticks);
    }


    public void debug() {
    }
}
