package org.firstinspires.ftc.teamcode.moonshine;

public abstract class Subsystem extends Command {
    protected Subsystem(Command[] children) {
        super(children);
    }

    abstract void onInitStart();
    abstract void onInitTick();
    abstract void onInitEnd();

    void initStep() {
        if(state == State.NOT_STARTED) {
            hydrateFields();
            onInitStart();
            state = State.STARTED;
        }
        else {
            onInitTick();
            state = State.TICKING;
        }
    }
    void initEnd() {
        onInitEnd();
        state = State.ENDED;
    }
}
