package org.firstinspires.ftc.teamcode.tasks

class InlineTask(private val task: () -> Unit) : Task() {
    override fun run() {
        task()
        state = TaskState.FINISHED
    }
}