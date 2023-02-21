package org.firstinspires.ftc.teamcode.moonshine;

public abstract class Subsystem extends Command {
    public Subsystem[] childrenSubsystems;

    protected Subsystem() {
        super();
    }

    protected abstract void onInitStart();
    protected abstract void onInitTick();
    protected abstract void onInitEnd();

    public void initStep() {
        if(state == State.NOT_STARTED) {
            state = State.STARTED;
            hydrateFields();
            onInitStart();
        }
        else {
            state = State.TICKING;
            onInitTick();
        }
    }
    public void initEnd() {
        if(!isRunning())
            return;

        state = State.ENDED;
        for(Subsystem child : childrenSubsystems) {
            child.initEnd();
        }
        onInitEnd();
    }
}
