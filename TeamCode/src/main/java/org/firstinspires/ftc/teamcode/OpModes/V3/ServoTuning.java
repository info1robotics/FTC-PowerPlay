package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class ServoTuning extends LinearOpMode {
    public static double servoPosition1 = 0;
    public static double servoPosition2 = 0;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Servo servo = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");

        while (opModeIsActive()) {
            telemetry.addData("Servo 1 Position", servo.getPosition());
            telemetry.addData("Servo 2 Position", servo2.getPosition());
            servo.setPosition(servoPosition1);
            servo2.setPosition(servoPosition2);

            telemetry.update();
        }
    }
}
