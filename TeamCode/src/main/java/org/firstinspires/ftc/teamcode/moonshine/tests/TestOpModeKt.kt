package org.firstinspires.ftc.teamcode.moonshine.tests

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.moonshine.SharedVar
import org.firstinspires.ftc.teamcode.moonshine.dsl.*
import java.util.function.IntSupplier
import java.util.function.Supplier

class TestOpModeKt : OpMode() {

    val detectionCase: Supplier<Int> by InjectSharedVar()
    val motor: DcMotor by InjectHardware()

    override fun init() {
        val routine = commandRoot {
            switch(detectionCase) {
                serial {  } // caz1
                serial {  } // caz2
                serial {  } // caz3
            }
        }
    }

    override fun loop() {
    }
}