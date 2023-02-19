package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;
import org.firstinspires.ftc.teamcode.moonshine.MoonshineException;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

public class SwitchCommand extends Command {

    public static final int INVALID = -1;

    int lastResult;
    private final Supplier<Integer> switchResult;

    public SwitchCommand(Supplier<Integer> switchResult, Command... children) {
        super(children);
        this.switchResult = switchResult;
    }

    @Override
    protected void onStart() {
        lastResult = INVALID;
        stepThroughSelected();
    }

    @Override
    protected void onTick() {
        stepThroughSelected();
    }

    @Override
    protected void onEnd() {
        int currentResult = getCurrentResult();
        if(lastResult != INVALID)
            children[currentResult].end();
    }

    void stepThroughSelected() {
        int currentResult = getCurrentResult();
        if(lastResult != INVALID && lastResult != currentResult) {
            children[lastResult].end();
        }
        if(currentResult != INVALID) {
            children[currentResult].step();
        }

        lastResult = currentResult;
    }

    int getCurrentResult() {
        int currentResult = switchResult.get();
        if(currentResult != INVALID && (currentResult < 0 || currentResult >= children.length))
            throw new MoonshineException(this, "SwitchCommand: switch result out of children array bounds");
        return currentResult;
    }
}
