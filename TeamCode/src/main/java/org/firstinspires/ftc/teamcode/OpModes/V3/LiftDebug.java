package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Extendo;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

@TeleOp
public class LiftDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
        Extendo extendo = new Extendo(this);
        lift.resetEncoders();

        GamepadEx gamepad_2 = new GamepadEx(gamepad2);

        int targetHeight = 0;

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            if (gamepad2.dpad_up) {
                targetHeight += 10;
            }
            if (gamepad2.dpad_down) {
                targetHeight -= 10;
            }

            if (gamepad_2.getButtonDown("a")) {
                extendo.setState(Extendo.ExtendoState.FULL);
            }

            if (gamepad_2.getButtonDown("b")) {
                extendo.setState(Extendo.ExtendoState.RETRACTED);
            }

            if (gamepad_2.getButtonDown("dpad_down")) {
                targetHeight = 200;
            }

            if (gamepad_2.getButtonDown("dpad_left")) {
                targetHeight = 500;
            }

            if (gamepad_2.getButtonDown("dpad_up")) {
                targetHeight = 1700;
            }

            if (gamepad_2.getButtonDown("dpad_right")) {
                targetHeight = 800;
            }

            if (targetHeight < 0) targetHeight = 0;
            if (targetHeight > 2000) targetHeight = 2000;


            lift.setHeight(targetHeight, 1);

            telemetry.addData("Lift left", lift.liftLeft.getCurrentPosition());
            telemetry.addData("Lift right", lift.liftRight.getCurrentPosition());
            gamepad_2.update();
            telemetry.update();
        }
    }
}
