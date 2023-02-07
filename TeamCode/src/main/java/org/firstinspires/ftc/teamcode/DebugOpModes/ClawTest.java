package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    Servo servoleft, servoright;
    servoleft = hardwareMap.get(Servo.class, "ServoLeft");
    servoright = hardwareMap.get(Servo.class, "ServoRight");
    servoleft.setPosition(1.0);
    servoright.setPosition(0.0);
    waitForStart();
    while(opModeIsActive()){
        servoleft.setPosition(0.0);
        servoright.setPosition(1.0);
        telemetry.update();
        }
    }
}
