package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.GoToAngleCommand
import org.firstinspires.ftc.teamcode.commands.GoToLevelCommand
import org.firstinspires.ftc.teamcode.moonshine.PrincipledTeleOpMode
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.ContinuousReuseCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.ParallelCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem

@TeleOp
class TurretDebug : PrincipledTeleOpMode() {
    private val linkageSubsystem = SharedSubsystem(LinkageSubsystem::class.java, LinkageSubsystem())
    private val turretSubsystem = SharedSubsystem(TurretSubsystem::class.java, TurretSubsystem())

    override fun getSubsystems(): ContinuousReuseCommand = newContinuous {
        add(linkageSubsystem)
        add(turretSubsystem)
    }

    override fun getInitRoutine(): SerialCommand = newSerial {  }

    override fun getControlScheme(): ContinuousReuseCommand = newContinuous {  }

    override fun getStartRoutine(): SerialCommand = newSerial {
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(GoToLevelCommand(LinkageSubsystem.LOW_LEVEL, .15))
            serial {
                sleep(200)
                run(GoToAngleCommand(90.0, .1))
            }
        }
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(GoToLevelCommand(LinkageSubsystem.MID_LEVEL, .15))
            run(GoToAngleCommand(0.0, .1))
        }
        parallel(ParallelCommand.Behavior.DEFAULT) {
            run(GoToLevelCommand(LinkageSubsystem.HIGH_LEVEL, .15))
            run(GoToAngleCommand(-90.0, .1))
        }
    }
}