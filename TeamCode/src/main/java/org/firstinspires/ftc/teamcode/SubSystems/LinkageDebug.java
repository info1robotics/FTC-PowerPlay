package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp
public class LinkageDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Linkage linkage = new Linkage(this);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Tick Count ", linkage.linkageLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
