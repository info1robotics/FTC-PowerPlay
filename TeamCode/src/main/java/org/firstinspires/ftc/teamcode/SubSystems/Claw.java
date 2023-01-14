package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Claw {
    public static final double SERVO_ARMED = 0.0;
    public static final double SERVO_READY = 1.0;
    public static boolean state;
    public Servo clawRight;
    public Servo clawLeft;

    // Activate the servo on initialization
    public Claw(LinearOpMode opMode) {
        state = true;
        clawLeft = opMode.hardwareMap.get(Servo.class, "ClawLeft");
        clawRight = opMode.hardwareMap.get(Servo.class, "ClawRight");
        setState(state);
    }

    // Simple toggle command for the claw mechanism (might be replaced in the near future)
    public void toggle() {
        state = !state;
        if (!state) {
            clawRight.setPosition(0.0);
            clawLeft.setPosition(0.57);
        } else {
            clawRight.setPosition(0.5);
            clawLeft.setPosition(0.25);
        }
    }

    public void setState(boolean state) {
        if (state) {
            clawRight.setPosition(0.0);
            clawLeft.setPosition(0.57);
        } else {
            clawRight.setPosition(1.0);
            clawLeft.setPosition(0.0);
        }
    }
}
