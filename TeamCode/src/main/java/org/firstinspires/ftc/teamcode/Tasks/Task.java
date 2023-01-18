package org.firstinspires.ftc.teamcode.Tasks;

import org.firstinspires.ftc.teamcode.OpModes.AutoOpMode;

public abstract class Task {
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    public boolean isRunning() {
        return state == State.RUNNING;
    }

    public enum State {
        DEFAULT,
        RUNNING,
        FINISHED,
    }

    public State state = State.DEFAULT;
    public AutoOpMode context;

    public void run() {};
    public void tick() {};

    public void start(AutoOpMode context) {
        this.context = context;
        state = State.RUNNING;
        run();
    }
}