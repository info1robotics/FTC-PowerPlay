package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

@TeleOp
public class LiftDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
//        Extendo extendo = new Extendo(this);
        lift.resetEncoders();

        GamepadEx gamepad_2 = new GamepadEx(gamepad2);

        int targetHeight = 0;
//        lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
//
            if (gamepad_2.getButtonDown("dpad_right")) {
                targetHeight = Lift.HIGH_POS;
            }

//
            if (gamepad_2.getButtonDown("dpad_up")) {
                targetHeight += 70;
            }

            if (gamepad_2.getButtonDown("dpad_down")) {
                targetHeight -= 70;
            }

//
            if (targetHeight < 0) targetHeight = 0;
//            if (targetHeight > 100) targetHeight = 1100;
//
//
            lift.setHeight(targetHeight, 1);

            telemetry.addData("Lift left", lift.liftLeft.getCurrentPosition());
            telemetry.addData("Lift right", lift.liftRight.getCurrentPosition());
            telemetry.addData("Target", targetHeight);
            gamepad_2.update();
            telemetry.update();
        }
    }
}
