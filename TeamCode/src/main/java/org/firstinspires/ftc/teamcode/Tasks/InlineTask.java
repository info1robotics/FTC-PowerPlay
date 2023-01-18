package org.firstinspires.ftc.teamcode.Tasks;

/**
 * Runs the given block synchronously.
 * Intended for one-liners.
 */
public class InlineTask extends Task {
    private final Runnable runnable;
    public InlineTask(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void run() {
        System.out.println("InlineTask was ran.");
        runnable.run();
        state = State.FINISHED;
    }
}
