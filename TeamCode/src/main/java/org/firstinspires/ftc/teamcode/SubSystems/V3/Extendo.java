package org.firstinspires.ftc.teamcode.SubSystems.V3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Extendo {
    public Servo extendoLeft;
    public Servo extendoRight;

    public Extendo(LinearOpMode opMode) {
        extendoLeft = opMode.hardwareMap.get(Servo.class, "ExtendoLeft");
        extendoRight = opMode.hardwareMap.get(Servo.class, "ExtendoRight");
        extendoRight.setDirection(Servo.Direction.REVERSE);
        extendoLeft.setDirection(Servo.Direction.REVERSE);
    }

    public void setState(ExtendoState state) {
        switch (state) {
            case RETRACTED: // 90
                extendoLeft.setPosition(1.0);
                extendoRight.setPosition(0.90);
                break;
            case FULL: // 65
                extendoLeft.setPosition(0.62);
                extendoRight.setPosition(0.55);
        }
    }


    public enum ExtendoState {
        RETRACTED, FULL
    }
}
