package org.firstinspires.ftc.teamcode.moonshine;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.moonshine.annotations.RequireHardware;

import java.lang.reflect.Field;


public abstract class Command {

    public enum State {
        NOT_STARTED,
        STARTED,
        TICKING,
        ENDED
    }

    State state = State.NOT_STARTED;
    Command parent = null;
    public Command[] children;


    protected Command(Command... children) {
        this.children = children;
        for (Command child : children) {
            child.setParent(this);
        }
    }

    protected void hydrateFields() {
        for(Field field : getClass().getDeclaredFields()) {
            RequireHardware rh = field.getAnnotation(RequireHardware.class);
            if(rh != null && !rh.hardwareName().isEmpty()) {
                field.setAccessible(true);
                try {
                    field.set(this, CommandEnv.getInstance().hardwareMap.get(field.getType(), rh.hardwareName()));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    protected abstract void onStart();
    protected abstract void onTick();
    protected abstract void onEnd();

    /**
     * In contrast to last year's version, this one only exposes a step() function
     * that automatically calls the either start() or tick() depending on current
     * state, and the action must call end() by itself in either of those, so it
     * now manages its own lifecycle fully and prevents lots of duplicated code.
     */

    public void step() {
        if(notStarted()) {
            state = State.STARTED;
            hydrateFields();
            onStart();
        } else if(isRunning()) {
            state = State.TICKING;
            onTick();
        }
    }

    public void end() {
        if(!isRunning())
            return;

        state = State.ENDED;
        for(Command child : children) {
            child.end();
        }
        onEnd();
    }

    public void reuse() {
        if(isRunning())
            throw new MoonshineException(this, "Cannot reuse a command that's still running!");

        state = State.NOT_STARTED;
        for(Command child : children) {
            child.reuse();
        }
    }

    public State getState() {
        return state;
    }

    public boolean notStarted() {
        return state == State.NOT_STARTED;
    }
    public boolean isRunning() {
        return state == State.STARTED || state == State.TICKING;
    }
    public boolean hasEnded() {
        return state == State.ENDED;
    }

    public Command getParent() {
        return parent;
    }
    public void setParent(Command parent) {
        this.parent = parent;
    }
}
