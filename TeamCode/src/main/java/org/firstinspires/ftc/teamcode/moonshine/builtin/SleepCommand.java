package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public class SleepCommand extends Command {
    long startTime;
    final long durationMs;

    public SleepCommand(long durationMs) {
        this.durationMs = durationMs;
    }

    @Override
    protected void onStart() {
        startTime = System.currentTimeMillis();
    }
    @Override
    protected void onTick() {
        if(startTime + durationMs < System.currentTimeMillis())
            end();
    }
    @Override
    protected void onEnd() {

    }
}
