package org.firstinspires.ftc.teamcode.commands

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.builtin.BlueprintCommand
import org.firstinspires.ftc.teamcode.moonshine.extensions.InjectHardware
import org.firstinspires.ftc.teamcode.moonshine.extensions.inline
import org.firstinspires.ftc.teamcode.moonshine.extensions.newSerial

class SetClawCommand(val clawState: ClawState) : BlueprintCommand() {
    enum class ClawState(val posLeft: Double, val posRight: Double) {
        OPEN(0.675, 0.325),
        CLOSED(1.0, 0.0)
    }

    val clawServoLeft by InjectHardware<Servo>("ClawLeft")
    val clawServoRight by InjectHardware<Servo>("ClawRight")

    override fun getBlueprint(): Command = newSerial {
        inline {
            clawServoLeft.position = clawState.posLeft
            clawServoRight.position = clawState.posRight
        }
    }


}