package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

import java.util.Arrays;
import java.util.function.Supplier;

public class TriggerCommand extends Command {

    private final Supplier<Boolean> cond;
    private boolean triggered;

    public TriggerCommand(Supplier<Boolean> cond, Command... oneChild) {
        this.children = oneChild;
        this.cond = cond;
    }

    @Override
    protected void onStart() {
        triggered = false;
    }

    @Override
    protected void onTick() {
        if(cond.get()) triggered = true;
        if(triggered) stepThroughChildren();
        if(triggered && Arrays.stream(children).allMatch(Command::hasEnded))
            end();
    }

    @Override
    protected void onEnd() {

    }

    private void stepThroughChildren() {
        if(children.length > 0)
            children[0].step();
    }
}
