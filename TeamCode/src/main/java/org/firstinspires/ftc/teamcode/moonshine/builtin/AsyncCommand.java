package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

/** DANGER : USE WITH CAUTION **/
public class AsyncCommand extends Command {
    private final Runnable runnable;
    Thread t;

    public AsyncCommand(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    protected void onStart() {
        t = new Thread(runnable);
        t.start();
    }

    @Override
    protected void onTick() {

    }

    @Override
    protected void onEnd() {
        t.interrupt();
    }
}
