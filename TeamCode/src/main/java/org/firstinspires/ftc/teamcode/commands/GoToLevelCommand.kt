package org.firstinspires.ftc.teamcode.commands

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.moonshine.OpModeType
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem

class GoToLevelCommand(var targetPosition: Int, var speed: Double = 0.5): Command() {
    val linkageSubsystem = SharedSubsystem(LinkageSubsystem::class.java)

    init {
        if (targetPosition > LinkageSubsystem.LINKAGE_MAX) targetPosition = LinkageSubsystem.LINKAGE_MAX
        if (targetPosition < LinkageSubsystem.LINKAGE_MIN) targetPosition = LinkageSubsystem.LINKAGE_MIN
    }

    override fun onStart() {
        linkageSubsystem.value.apply {
            setTargetPosition(targetPosition)
            setMode(DcMotor.RunMode.RUN_TO_POSITION)
        }
    }

    override fun onTick() {
        if (opModeType == OpModeType.AUTONOMOUS && targetPosition > 550)
            speed = 0.15

        linkageSubsystem.value.let {
            if (it.hasReachedTarget) {
                end()
            } else {
                it.setPowers(speed)
            }
        }
    }

    override fun onEnd() {
    }
}