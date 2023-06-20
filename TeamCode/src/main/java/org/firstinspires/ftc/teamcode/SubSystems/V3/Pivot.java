package org.firstinspires.ftc.teamcode.SubSystems.V3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
@Config
public class Pivot {
    final Servo pivotLeft;
    final Servo pivotRight;

    public static double PIVOT_SCORE = 0.7; // TODO: Find values

    public Pivot(LinearOpMode opMode) {
        pivotLeft = opMode.hardwareMap.get(Servo.class, "PivotLeft");
        pivotRight = opMode.hardwareMap.get(Servo.class, "PivotRight");
        pivotRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setCollect() {
        pivotLeft.setPosition(0.0);
        pivotRight.setPosition(0.0);
        Claw.getInstance().rotateDefault();
        Claw.getInstance().open();
    }

    public void setScore() {
        pivotLeft.setPosition(PIVOT_SCORE);
        pivotRight.setPosition(PIVOT_SCORE);
        Claw.getInstance().rotateFlipped();
//        Claw.getInstance().close();
    }

    public void setManual(double position) {
        pivotLeft.setPosition(position);
        pivotRight.setPosition(position);
    }

    public void toggle() {
        if (pivotLeft.getPosition() == 0.0) {
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
