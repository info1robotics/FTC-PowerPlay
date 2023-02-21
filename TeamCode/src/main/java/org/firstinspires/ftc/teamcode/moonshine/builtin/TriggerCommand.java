package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

import java.util.Arrays;
import java.util.function.Supplier;

public class TriggerCommand extends Command {

    private final Supplier<Boolean> cond;
    private boolean triggered;

    public TriggerCommand(Supplier<Boolean> cond) {
        this.cond = cond;
    }

    @Override
    protected void onStart() {
        triggered = false;
        yes();
    }

    @Override
    protected void onTick() {
        yes();
    }

    @Override
    protected void onEnd() {

    }

    private void yes() {
        if(cond.get()) triggered = true;
        if(triggered) stepThroughChildren();
        if(Arrays.stream(childrenCommands).allMatch(Command::hasEnded))
            end();
    }

    private void stepThroughChildren() {
        for(Command child : childrenCommands) {
            child.step();
        }
    }
}
