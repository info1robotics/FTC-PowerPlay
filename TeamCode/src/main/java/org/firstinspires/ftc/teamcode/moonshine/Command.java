package org.firstinspires.ftc.teamcode.moonshine;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public Command[] children;

    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected OpModeType opModeType;

    protected Command(Command... children) {
        this.children = children;
    }

    protected void hydrateFields() {
        hardwareMap = CommandEnv.getInstance().eventLoop.getOpModeManager().getActiveOpMode().hardwareMap;
        telemetry   = CommandEnv.getInstance().eventLoop.getOpModeManager().getActiveOpMode().telemetry;
        opModeType  = CommandEnv.getInstance().eventLoop.getOpModeManager().getActiveOpMode().getClass().isAnnotationPresent(Autonomous.class)?
            OpModeType.AUTONOMOUS : OpModeType.TELEOP;


        for(Field field : getClass().getDeclaredFields()) {
            RequireHardware rh = field.getAnnotation(RequireHardware.class);
            if(rh != null && !rh.hardwareName().isEmpty()) {
                field.setAccessible(true);
                try {
                    field.set(this, hardwareMap.get(field.getType(), rh.hardwareName()));
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
}
