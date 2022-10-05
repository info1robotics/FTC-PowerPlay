package org.firstinspires.ftc.teamcode.tasks

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

enum class TaskState {
    RUNNING,
    FINISHED,
}

abstract class Task {
    var state = TaskState.RUNNING
    open fun tick() {}
    abstract fun run()
    lateinit var context: LinearOpMode
    fun start(ctx: LinearOpMode) {
        state = TaskState.RUNNING
        context = ctx
        run()
    }
    fun isRunning() = state == TaskState.RUNNING
    fun isFinished() = state == TaskState.FINISHED
}