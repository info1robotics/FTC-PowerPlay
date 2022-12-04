package org.firstinspires.ftc.teamcode.Tasks

class ParallelTasks(vararg tasks: Task) : Task() {
    var childTasks = tasks
    var finishedTasks = 0

    override fun tick() {
        finishedTasks = 0
        childTasks.forEach {
            if (it.state == TaskState.RUNNING) {
                it.tick()
            } else if (it.state == TaskState.FINISHED) {
                finishedTasks++
            }
        }
        if (finishedTasks == childTasks.size) {
            state = TaskState.FINISHED
        }
    }

    override fun run() {
        for (task in childTasks) {
            task.start(this.context)
        }
    }
}