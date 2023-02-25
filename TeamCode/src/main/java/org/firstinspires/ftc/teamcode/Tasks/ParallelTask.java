package org.firstinspires.ftc.teamcode.Tasks;

/**
 * Starts each child in parallel.
 * Ends when all children finish.
 */
public class ParallelTask extends CompoundTask {
    public ParallelTask(Task... children) {
        super(children);
    }

    @Override
    public void tick() {
        int finishedTasks = 0;
        for (Task child : children) {
            if (child.isFinished()) {
                finishedTasks++;
                continue;
            }
            child.tick();
        }

        if (finishedTasks == children.length) {
            state = State.FINISHED;
        }
    }

    @Override
    public void run() {
        for (Task task : children) task.start(context);
    }
}
