package org.firstinspires.ftc.teamcode.tasks

import org.firstinspires.ftc.teamcode.tasks.Task
import org.firstinspires.ftc.teamcode.tasks.TaskState

class SleepTask(private val ms: Int) : Task() {
    private var timeAtRun: Long = -1
    override fun tick() {
        val currentTime = System.currentTimeMillis()
        if (currentTime - timeAtRun > ms) {
            state = TaskState.FINISHED
            return
        }
    }

    override fun run() {
        timeAtRun = System.currentTimeMillis()
    }
}