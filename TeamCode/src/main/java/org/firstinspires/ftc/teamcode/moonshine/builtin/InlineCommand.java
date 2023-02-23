package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public class InlineCommand extends Command {

    private final Runnable runnable;

    public InlineCommand(Runnable runnable) {
        this.runnable = runnable;
    }
    @Override
    protected void onStart() {
        runnable.run();
        end();
    }

    @Override
    protected void onTick() {
        end();
    }

    @Override
    protected void onEnd() {

    }
}
