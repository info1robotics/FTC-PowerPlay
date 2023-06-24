package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V3.Extendo;

@TeleOp
public class ExtendoDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Extendo extendo = new Extendo(this);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.dpad_up) {
                extendo.setState(Extendo.ExtendoState.FULL);
            }
            if (gamepad1.dpad_down) {
                extendo.setState(Extendo.ExtendoState.RETRACTED);
            }
        }
    }
}
