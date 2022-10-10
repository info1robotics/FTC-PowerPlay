package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Common.StateOpMode

@TeleOp
class TestOp : StateOpMode() {
    override fun onLoop() {
        mecanum.vectorMove(
                -gamepad1.left_stick_x.toDouble(),
                gamepad1.left_stick_y.toDouble(),
                gamepad1.left_trigger - gamepad1.right_trigger.toDouble(),
                if(gamepad1.right_bumper) 0.5 else 1.0
        )
    }
}