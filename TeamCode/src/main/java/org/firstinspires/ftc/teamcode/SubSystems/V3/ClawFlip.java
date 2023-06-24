package org.firstinspires.ftc.teamcode.SubSystems.V3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawFlip {
    final Servo clawFlip;

    public static final double ROTATE_COLLECT = 0.0;
    public static final double ROTATE_SCORE = 0.72;

    public ClawFlip(LinearOpMode opMode) {
        clawFlip = opMode.hardwareMap.get(Servo.class, "ClawFlip");
    }

    public void setCollect() {
        clawFlip.setPosition(ROTATE_COLLECT);
    }

    public void setScore() {
        clawFlip.setPosition(ROTATE_SCORE);
    }

    public void toggle() {
        if (clawFlip.getPosition() == ROTATE_COLLECT) {
            clawFlip.setPosition(ROTATE_SCORE);
        } else {
            clawFlip.setPosition(ROTATE_COLLECT);
        }
    }
}
