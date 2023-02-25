package org.firstinspires.ftc.teamcode.SubSystems.V2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo clawLeft, clawRight;

    public double LEFT_OPEN = 1.0;
    public double LEFT_CLOSED = 0.0;
    public double RIGHT_OPEN = 0.0;
    public double RIGHT_CLOSED = 1.0;
    public static boolean isClosed;
    public enum states{
        OPEN,
        CLOSED
    }

    public Claw(LinearOpMode opMode) {
        clawLeft = opMode.hardwareMap.get(Servo.class, "ClawLeft");
        clawRight = opMode.hardwareMap.get(Servo.class, "ClawRight");
        setState(states.OPEN);
    }

    public void setState(states state) {
        if (state == states.OPEN) {
            // Opens the Claw
            clawRight.setPosition(RIGHT_OPEN);
            clawLeft.setPosition(LEFT_OPEN);
        } else if (state == states.CLOSED) {
            // Closes the Claw
            clawRight.setPosition(RIGHT_CLOSED);
            clawLeft.setPosition(LEFT_CLOSED);
        }
    }
    public void toggle() {
        isClosed = !isClosed;
        if (!isClosed) {
            // Opens the Claw
            clawRight.setPosition(RIGHT_OPEN);
            clawLeft.setPosition(LEFT_OPEN);
        } else {
            // Closes the Claw
            clawRight.setPosition(RIGHT_CLOSED);
            clawLeft.setPosition(LEFT_CLOSED);
        }
    }
}
