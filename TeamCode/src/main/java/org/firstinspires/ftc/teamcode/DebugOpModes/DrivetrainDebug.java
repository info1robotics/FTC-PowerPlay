package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
@Disabled
@TeleOp(name="Drivetrain Debug")
public class DrivetrainDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this.hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            drivetrain.vectorMove(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x + (gamepad1.left_trigger - gamepad1.right_trigger),
//                    gamepad1.right_stick_x,
                    gamepad1.right_bumper ? 0.75 : 1.0
            );
        }
    }
}
