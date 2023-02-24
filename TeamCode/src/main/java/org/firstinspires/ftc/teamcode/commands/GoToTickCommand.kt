package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem
import java.util.function.Supplier

class GoToTickCommand(private val targetPosition: Supplier<Int>, private val speed: Double = 0.1) : Command() {
    val turretSubsystem = SharedSubsystem(TurretSubsystem::class.java)

    override fun onStart() {
        turretSubsystem.value.targetPosition = targetPosition.get()
        turretSubsystem.value.power = speed
    }

    override fun onTick() {
        if (turretSubsystem.value.hasReachedTarget) end()
    }

    override fun onEnd() {
        turretSubsystem.value.targetPosition = turretSubsystem.value.currentPosition
    }
}