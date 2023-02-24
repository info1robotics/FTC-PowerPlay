package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.PrincipledTeleOpMode
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.ContinuousReuseCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

@TeleOp
class BrakeDebug : PrincipledTeleOpMode() {
    private val turretSubsystem = SharedSubsystem(TurretSubsystem::class.java, TurretSubsystem())

    override fun getSubsystems(): ContinuousReuseCommand = newContinuous {
        add(turretSubsystem)
    }

    override fun getInitRoutine(): SerialCommand = newSerial {
    }

    override fun getControlScheme(): ContinuousReuseCommand = newContinuous {

    }

    override fun getStartRoutine(): SerialCommand = newSerial {
        run(object: Command() {
            override fun onStart() {}
            override fun onTick() {
                turretSubsystem.value.noAutoBrakes = true
                if(gamepad1.triangle) {
                    println("AAAAAAAAAAA")
                    turretSubsystem.value.disengageBrake()
                    turretSubsystem.value.disengageSuperBrake()
                }
                if(gamepad1.square) {
                    println("BBBBBBBBBBBB")
                    turretSubsystem.value.engageBrake()
                    turretSubsystem.value.engageSuperBrake()
                }
            }
            override fun onEnd() {}
        })
    }
}