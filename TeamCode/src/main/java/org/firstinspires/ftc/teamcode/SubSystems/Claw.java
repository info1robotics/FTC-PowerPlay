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
    public static enum states{
        OPEN,
        CLOSED
    }

    // Activate the servo on initialization
    public Claw(LinearOpMode opMode) {
        clawLeft = opMode.hardwareMap.get(Servo.class, "ClawLeft");
        clawRight = opMode.hardwareMap.get(Servo.class, "ClawRight");
        setState(states.OPEN);
    }

    // Simple toggle command for the claw mechanism (might be replaced in the near future)
    public void toggle() {
        state = !state;
        if (!state) {
            //closed
            clawRight.setPosition(0.1);
            clawLeft.setPosition(0.6);
        } else {
            clawRight.setPosition(0.45);
            clawLeft.setPosition(0.3);
        }
    }

    public void setState(states state) {

        if (state == states.OPEN) {
            clawRight.setPosition(0.1);
            clawLeft.setPosition(0.6);
        } else if (state == states.CLOSED){
            clawRight.setPosition(0.45);
            clawLeft.setPosition(0.3);
        }
    }
}
