package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public class SerialCommand extends Command {
    int current = 0;

    public SerialCommand(Command... children) {
        super(children);
    }

    @Override
    protected void onStart() {
        current = 0;
    }
    @Override
    protected void onTick() {
        stepThroughChildren();
    }
    @Override
    protected void onEnd() {
    }

    void stepThroughChildren() {
        if(current < children.length) {
            if(children[current].hasEnded())
                current++;
            else
                children[current].step();
        } else end();
    }
}
