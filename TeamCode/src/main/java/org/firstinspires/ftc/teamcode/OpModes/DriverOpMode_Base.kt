package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Common.Mecanum
import org.firstinspires.ftc.teamcode.Common.GamepadEx

//@Disabled
@TeleOp
class DriverOpMode_Base : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val mecanum = Mecanum(hardwareMap)
        val g1 = GamepadEx(gamepad1)
        waitForStart()
        while (opModeIsActive()) {
            g1.update()
            mecanum.vectorMove(-gamepad1.left_stick_x.toDouble(),
                    gamepad1.left_stick_y.toDouble(), (
                    gamepad1.left_trigger - gamepad1.right_trigger).toDouble(),
                    if (gamepad1.right_bumper) 0.5 else 1.0
            )
            telemetry.update()
        }
    }
}