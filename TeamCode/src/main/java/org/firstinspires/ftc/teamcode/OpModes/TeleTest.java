package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.GamepadEx;
import org.firstinspires.ftc.teamcode.Common.Mecanum;

//@Disabled
@TeleOp
public class TeleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Mecanum mecanum = new Mecanum(this.hardwareMap);
        GamepadEx g1 = new GamepadEx(gamepad1);

        waitForStart();

        while(opModeIsActive())
        {

            g1.update();
            mecanum.vectorMove(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    gamepad1.right_bumper ? 0.5 : 1.0
            );
            telemetry.update();
        }
    }
}