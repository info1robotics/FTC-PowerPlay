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
    public Servo clawSingle;
    public static final double SERVO_ARMED = 0.0;
    public static final double SERVO_READY = 1.0;

    // Simple toggle command for the claw mechanism (might be replaced in the near future)
    public void TOGGLE(){
        if(clawSingle.getPosition()==SERVO_ARMED) clawSingle.setPosition(SERVO_READY);
        else clawSingle.setPosition(SERVO_ARMED);
    }

    // Activate the servo on initialization
    public Claw(LinearOpMode opMode) {
        clawSingle = opMode.hardwareMap.get(Servo.class, "Claw");
        clawSingle.setPosition(SERVO_ARMED);
    }
}
