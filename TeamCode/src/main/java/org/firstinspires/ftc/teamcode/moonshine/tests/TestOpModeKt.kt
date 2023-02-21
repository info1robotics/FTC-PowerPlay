package org.firstinspires.ftc.teamcode.moonshine.tests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.moonshine.extensions.*

@TeleOp
class TestOpModeKt : OpMode() {

    val routine = newAutoRoutine {
        var switcher = 0
        switch({ switcher }) {
            serial { // Child 0
                inline { telemetry.addLine("Hi from Serial1! Waiting 3sec then switching!") }
                sleep(1000)
                inline { telemetry.addLine("3!") }
                sleep(1000)
                inline { telemetry.addLine("2!") }
                sleep(1000)
                inline { telemetry.addLine("1!") }
                sleep(1000)
                inline { switcher = 1 }
            }

            serial { // Child 1
                inline { telemetry.addLine("Hi from Serial2! Waiting 1sec then switching!") }
                sleep(1000)
                inline { switcher = 0 }
            }
        }
    }

    override fun init() {
    }

    override fun loop() {
        routine.step()
        telemetry.update()
    }
}