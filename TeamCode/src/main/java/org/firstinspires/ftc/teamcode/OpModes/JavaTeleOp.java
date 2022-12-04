package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.GROUND_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.MID_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.HIGH_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.JUNCTION_LEVEL;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Linkage;

@TeleOp(name = "StandardTeleOp")
public class JavaTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Linkage linkage = new Linkage(this);
        Claw claw = new Claw(this);
        GamepadEx controller1 = new GamepadEx(gamepad1);

        waitForStart();

        while(opModeIsActive()){

            if(controller1.getButtonDown("a")) claw.TOGGLE();
            if(controller1.getButtonDown("dpad_right")) linkage.GO_TO_LEVEL(MID_LEVEL, 0.25);
            if(controller1.getButtonDown("dpad_down")) linkage.GO_TO_LEVEL(GROUND_LEVEL, 0.25);

            telemetry.update();
        }
    }
}
