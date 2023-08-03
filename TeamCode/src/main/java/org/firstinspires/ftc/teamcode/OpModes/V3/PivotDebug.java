package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V3.ClawFlip;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Extendo;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Pivot;

@TeleOp
public class PivotDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ClawFlip flip = new ClawFlip(this);
        flip.setCollect();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            telemetry.update();
        }
    }
}
