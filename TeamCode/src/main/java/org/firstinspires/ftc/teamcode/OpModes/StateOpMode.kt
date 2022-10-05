package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Common.Mecanum
import org.firstinspires.ftc.teamcode.tasks.InlineTask
import org.firstinspires.ftc.teamcode.tasks.SyncTasks
import org.firstinspires.ftc.teamcode.tasks.Task
import org.firstinspires.ftc.teamcode.tasks.TaskState
import kotlin.Throws

abstract class StateOpMode : LinearOpMode() {
    private var state = 0
    lateinit var mecanum: Mecanum
    open val task: Task = InlineTask {}
    open fun onInit() {}
    open fun onLoop() {}
    open fun onEnd() {}
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        mecanum = Mecanum(this.hardwareMap)
        onInit()
        waitForStart()
        task.start(this)
        while (opModeIsActive()) {
            onLoop()
            if (task.state == TaskState.RUNNING) task.tick()
            telemetry.update()
        }
        onEnd()
    }
}