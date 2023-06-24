package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

@TeleOp
public class LiftManual extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            lift.setPower(gamepad1.left_stick_y);
        }
    }
}
