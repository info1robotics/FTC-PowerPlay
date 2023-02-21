package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public class ParallelCommand extends Command {
    private final boolean shorting;

    public ParallelCommand(boolean shorting, Command... children) {
        super(children);
        this.shorting = shorting;
    }

    @Override
    protected void onStart() {
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
        int endedChildren = 0;
        for(Command child : childrenCommands) {
            child.step();
            if(child.hasEnded()) {
                endedChildren++;
            }
        }
        if(endedChildren == childrenCommands.length || (endedChildren > 0 && shorting)) {
            end();
        }
    }
}
