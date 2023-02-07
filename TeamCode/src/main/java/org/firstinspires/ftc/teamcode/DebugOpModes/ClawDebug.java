package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;

//@Disabled
@TeleOp
public class ClawDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a) claw.toggle();

            // Get feedback from the servo internal sensor for potential debugging.
            telemetry.update();
        }
    }
}
