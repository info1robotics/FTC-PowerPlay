package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.GoToLevelCommand
import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.moonshine.PrincipledTeleOpMode
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.ContinuousReuseCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem

@TeleOp
class LinkageDebug : PrincipledTeleOpMode() {
    private val linkageSubsystem = SharedSubsystem(LinkageSubsystem::class.java, LinkageSubsystem())

    override fun getSubsystems(): ContinuousReuseCommand = newContinuous {
        add(linkageSubsystem)
    }

    override fun getInitRoutine(): SerialCommand = newSerial {  }

    override fun getControlScheme(): ContinuousReuseCommand = newContinuous {  }
    override fun getStartRoutine(): SerialCommand = newSerial {
        var case = 0
        switch({ case }) {
            serial {
                run(GoToLevelCommand(LinkageSubsystem.LOW_LEVEL, 0.1))
                sleep(2000)
                inline { case = 1 }
            }
            serial {
                run(GoToLevelCommand(LinkageSubsystem.MID_LEVEL, 0.1))
                sleep(2000)
                inline { case = 2 }
            }
            serial {
                run(GoToLevelCommand(LinkageSubsystem.HIGH_LEVEL, 0.1))
                sleep(2000)
                inline { case = 0 }
            }
        }
    }
}