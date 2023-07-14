package org.firstinspires.ftc.teamcode.SubSystems.V3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
@Config
public class Pivot {
    final Servo pivotLeft;
    final Servo pivotRight;

    public static boolean collect = false;

    public static double PIVOT_COLLECT = 0.05;
    public static double PIVOT_MID = 0.6;

    public static double PIVOT_SCORE = 0.6; // TODO: Find values

    public static int PIVOT_SLEEP = 565;

    public Pivot(LinearOpMode opMode) {
        pivotLeft = opMode.hardwareMap.get(Servo.class, "PivotLeft");
        pivotRight = opMode.hardwareMap.get(Servo.class, "PivotRight");
//        ServoImplEx a;
//        pivotRight.getController().getPwmStatus()
//        a.setPwmDisable();
        pivotRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setCollect() {
        pivotLeft.setPosition(PIVOT_COLLECT);
        pivotRight.setPosition(PIVOT_COLLECT);
        collect = true;
    }

    public void setHalf() {
        pivotLeft.setPosition(PIVOT_MID);
        pivotRight.setPosition(PIVOT_MID);
    }

    public void setScore() {
        pivotLeft.setPosition(PIVOT_SCORE);
        pivotRight.setPosition(PIVOT_SCORE);
        collect = false;
    }

    public boolean isCollect() {
        return collect;
    }

    public boolean isScore() {
        return !collect;
    }

    public void toggle() {
        if (collect) {
            setScore();
        } else {
            setCollect();
        }
    }

    public void debug() {
        telemetry.addData("Pivot Left", pivotLeft.getPosition());
        telemetry.addData("Pivot Right", pivotRight.getPosition());
    }

}
