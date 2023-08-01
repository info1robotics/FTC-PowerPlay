package org.firstinspires.ftc.teamcode.SubSystems.V3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private boolean closed = false;
    final Servo claw, clawFlip;

    public static final double CLAW_CLOSED = 0.0;
    public static final double CLAW_OPEN = 0.42;

    private static Claw instance;

    public static Claw getInstance() {
        return instance;
    }

    public Claw(LinearOpMode opMode) {
        claw = opMode.hardwareMap.get(Servo.class, "Claw");
        clawFlip = opMode.hardwareMap.get(Servo.class, "ClawFlip");
        instance = this;
    }


    public void open() {
        claw.setPosition(CLAW_OPEN);
        closed = false;
    }

    public void close() {
        claw.setPosition(CLAW_CLOSED);
        closed = true;
    }

    public void toggle() {
        if (closed) {
            open();
        } else {
            close();
        }
    }

    public void debug() {
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("ClawFlip", clawFlip.getPosition());
    }

}
