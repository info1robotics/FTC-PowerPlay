package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;

//@Disabled
@TeleOp
public class ClawDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a) claw.TOGGLE();
            telemetry.update();
        }
    }
}
