package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

class GoToAngleCommand(val angle: Double, val speed: Double) : Command() {
    val turretSubsystem = SharedSubsystem(TurretSubsystem::class.java)

    override fun onStart() {
        turretSubsystem.value.setTargetAngle(angle)
        turretSubsystem.value.power = speed
    }

    override fun onTick() {
        if (turretSubsystem.value.hasReachedTarget) end()
    }

    override fun onEnd() {
        turretSubsystem.value.targetPosition = turretSubsystem.value.currentPosition
    }
}