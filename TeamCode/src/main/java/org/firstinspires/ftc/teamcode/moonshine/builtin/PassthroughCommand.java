package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

import java.util.Arrays;

public class PassthroughCommand extends Command {

    private final boolean automaticPrepareForReuse;

    public PassthroughCommand(boolean automaticPrepareForReuse, Command... children) {
        super(children);
        this.automaticPrepareForReuse = automaticPrepareForReuse;
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
        for(Command child : childrenCommands) {
            child.step();
            if(child.hasEnded() && automaticPrepareForReuse)
                child.reuse();
        }
    }
//
//    public void cleanEnded() {
//        childrenCommands = (Command[]) Arrays.stream(childrenCommands)
//            .filter(command -> !command.hasEnded())
//            .toArray();
//    }
}
