package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LinkageDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Linkage linkage = new Linkage(this);
        waitForStart();
        while(opModeIsActive()){
            linkage.linkageRight.setPower(gamepad1.left_stick_x);
            linkage.linkageLeft.setPower(gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
