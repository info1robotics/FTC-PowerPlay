package org.firstinspires.ftc.teamcode.Tasks;

public class SyncTask extends CompoundTask {
    public int currentTask = 0;

    public SyncTask(Task... children) {
        super(children);
    }

    @Override
    public void tick() {
        if (children[currentTask].isFinished()) {
            currentTask++;
            if (currentTask >= children.length) {
                state = State.FINISHED;
                currentTask = 0;
                for (Task child : children) child.state = State.DEFAULT;
                return;
            }
            children[currentTask].start(this.context);
        } else if (children[currentTask].isRunning()) {
            children[currentTask].tick();
        }
    }

    @Override
    public void run() {
        if (children.length == 0) {
            state = State.FINISHED;
            return;
        }
        children[0].start(context);
    }
}
