package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Claw {
    public Servo clawRight;
    public Servo clawLeft;
    public static String STATE;
    public static final double SERVO_ARMED = 0.0;
    public static final double SERVO_READY = 1.0;

    // Simple toggle command for the claw mechanism (might be replaced in the near future)
    public void TOGGLE(){
        if(STATE == "OPEN"){
            clawRight.setPosition(0.5);
            clawLeft.setPosition(0.5);
            STATE = "CLOSED";
        }
        else{
            clawRight.setPosition(1.0);
            clawLeft.setPosition(0.0);
            STATE = "OPEN";
        }
    }

    public void SET_STATE(String state){
        if(state=="OPEN") {
            clawRight.setPosition(0.5);
            clawLeft.setPosition(0.5);
        } else{
            clawRight.setPosition(1.0);
            clawLeft.setPosition(0.0);
        }
    }

    // Activate the servo on initialization
    public Claw(LinearOpMode opMode) {
        STATE = "OPEN";
        clawLeft = opMode.hardwareMap.get(Servo.class, "ClawLeft");
        clawRight = opMode.hardwareMap.get(Servo.class, "ClawRight");
        SET_STATE(STATE);
    }
}
