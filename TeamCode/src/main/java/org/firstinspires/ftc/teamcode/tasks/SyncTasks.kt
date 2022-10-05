package org.firstinspires.ftc.teamcode.tasks

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.tasks.Task
import org.firstinspires.ftc.teamcode.tasks.TaskState

class SyncTasks(private vararg val childTasks: Task) : Task() {
    var currentTask = 0
    override fun tick() {
        childTasks[currentTask].tick()
        if (childTasks[currentTask].state == TaskState.FINISHED) {
            currentTask++
            if (currentTask >= childTasks.size) {
                state = TaskState.FINISHED
                return
            }
            childTasks[currentTask].start(this.context)
        }
    }

    override fun run() {
        if (childTasks.isEmpty()) {
            state = TaskState.FINISHED
            return
        }
        childTasks[0].start(this.context)
    }
}
