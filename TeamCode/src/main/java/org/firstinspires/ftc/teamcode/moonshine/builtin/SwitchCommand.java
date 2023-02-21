package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;
import org.firstinspires.ftc.teamcode.moonshine.MoonshineException;

import java.util.function.Supplier;

public class SwitchCommand extends Command {

    public static final int NONE = -1;

    int lastResult;
    private final Supplier<Integer> switchResult;

    public SwitchCommand(Supplier<Integer> switchResult, Command... children) {
        super(children);
        this.switchResult = switchResult;
    }

    @Override
    protected void onStart() {
        lastResult = NONE;
        stepThroughSelected();
    }

    @Override
    protected void onTick() {
        stepThroughSelected();
    }

    @Override
    protected void onEnd() {
    }

    void stepThroughSelected() {
        int currentResult = getCurrentResult();

        if(lastResult != currentResult) {
            if(lastResult != NONE) {
                childrenCommands[lastResult].end();
            }
            if(childrenCommands[currentResult].hasEnded())
                childrenCommands[currentResult].reuse();
        }

        if(currentResult != NONE) {
            childrenCommands[currentResult].step();
        } else {
            end();
        }

        lastResult = currentResult;
    }

    int getCurrentResult() {
        int currentResult = switchResult.get();
        if(currentResult != NONE && (currentResult < 0 || currentResult >= childrenCommands.length))
            throw new MoonshineException(
                this,
                String.format(
                    "SwitchCommand: switch result \"%d\" out of children array bounds",
                    currentResult)
            );

        return currentResult;
    }
}
