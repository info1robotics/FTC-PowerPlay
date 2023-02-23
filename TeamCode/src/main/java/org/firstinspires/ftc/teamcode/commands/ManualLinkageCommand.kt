package org.firstinspires.ftc.teamcode.commands

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem
import java.util.function.Supplier

class ManualLinkageCommand(var speed: Supplier<Double>) : Command() {
    private val linkageSubsystem = SharedSubsystem(LinkageSubsystem::class.java)

    override fun onStart() {

    }

    override fun onTick() {
        linkageSubsystem.value.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        linkageSubsystem.value.setPowers(speed.get())
    }

    override fun onEnd() {

    }
}