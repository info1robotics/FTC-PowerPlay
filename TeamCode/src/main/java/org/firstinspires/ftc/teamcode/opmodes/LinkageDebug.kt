package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.GoToLevelCommand
import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.extensions.*
import org.firstinspires.ftc.teamcode.subsystems.LinkageSubsystem

@TeleOp
class LinkageDebug : LinearOpMode() {

    val linkageSubsystem = SharedSubsystem(LinkageSubsystem::class.java, LinkageSubsystem())


    override fun runOpMode() {
        waitForStart()

        val routine = newSerial {
            var case = 0
            switch({ case }) {
                serial {
                    run(GoToLevelCommand(LinkageSubsystem.LOW_LEVEL))
                    sleep(2000)
                    inline { case = 1 }
                }
                serial {
                    run(GoToLevelCommand(LinkageSubsystem.MID_LEVEL))
                    sleep(2000)
                    inline { case = 2 }
                }
                serial {
                    run(GoToLevelCommand(LinkageSubsystem.HIGH_LEVEL))
                    sleep(2000)
                    inline { case = 0 }
                }
            }
        }

        while(opModeIsActive()) {
            routine.step()
        }
        routine.end()
    }
}