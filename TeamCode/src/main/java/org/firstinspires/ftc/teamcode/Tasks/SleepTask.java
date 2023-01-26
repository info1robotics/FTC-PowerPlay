package org.firstinspires.ftc.teamcode.Tasks;

/**
 * Waits for the given amount of time.
 * Unless used in a synchronous task, this will not block the thread of execution.
 * @param ms Time to wait in milliseconds.
 */
public class SleepTask extends Task {
    public long timeAtRun = -1;
    public long ms;

    public SleepTask(long ms) {
        this.ms = ms;
    }

    @Override
    public void tick() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - timeAtRun > ms) state = State.FINISHED;

    }

    @Override
    public void run() {
        timeAtRun = System.currentTimeMillis();
    }
}
