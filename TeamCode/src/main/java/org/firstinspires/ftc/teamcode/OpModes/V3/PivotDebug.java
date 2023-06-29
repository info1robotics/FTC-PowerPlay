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
        Pivot pivot = new Pivot(this);
        pivot.setScore();
        waitForStart();
        GamepadEx gamepad_2 = new GamepadEx(gamepad2);
        pivot.setCollect();
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad_2.getButtonDown("a")) {
                pivot.setCollect();
            }
            if (gamepad_2.getButtonDown("x")) {
                pivot.setScore();
            }
            gamepad_2.update();
            telemetry.update();
        }
    }
}
