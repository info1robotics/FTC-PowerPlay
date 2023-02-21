package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public class ParallelCommand extends Command {
    private final Behavior behavior;

    public enum Behavior {
        DEFAULT,
        CONTINUOUS,
        SHORT
    }

    public ParallelCommand(Behavior behavior, Command... children) {
        super(children);
        this.behavior = behavior;
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
        for(Command child : children) {
            child.step();
            if(child.hasEnded()) {
                if(behavior != Behavior.CONTINUOUS) {
                    endedChildren++;
                } else {
                    child.reuse();
                }
            }
        }

        if(endedChildren == children.length || (endedChildren > 0 && behavior == Behavior.SHORT)) {
            end();
        }
    }
}
