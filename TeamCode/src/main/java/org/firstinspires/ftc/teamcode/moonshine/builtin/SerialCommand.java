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
        stepThroughChildren();
    }
    @Override
    protected void onTick() {
        stepThroughChildren();
    }
    @Override
    protected void onEnd() {
    }

    void stepThroughChildren() {
        if(current < childrenCommands.length) {
            if(childrenCommands[current].hasEnded())
                current++;
            else
                childrenCommands[current].step();
        } else end();
    }
}
