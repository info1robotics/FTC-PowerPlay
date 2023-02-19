package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

import java.util.Arrays;
import java.util.function.Predicate;

public class PassthroughCommand extends Command {

    public PassthroughCommand(Command... children) {
        super(children);
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
        for(Command child : children) {
            child.end();
        }
    }

    void stepThroughChildren() {
        for(Command child : children) {
            child.step();
        }
    }

    public void garbageCollect() {
        children = (Command[]) Arrays.stream(children)
            .filter(command -> !command.hasEnded())
            .toArray();
    }
}
