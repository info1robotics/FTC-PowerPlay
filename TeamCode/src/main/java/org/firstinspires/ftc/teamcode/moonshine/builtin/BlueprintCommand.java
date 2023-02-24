package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public abstract class BlueprintCommand extends Command {
    Command blueprint;

    public BlueprintCommand() {
        super();
    }

    @Override
    protected void onStart() {
        blueprint = getBlueprint();
    }

    @Override
    protected void onTick() {
        blueprint.step();
        if(blueprint.hasEnded())
             end();
    }

    @Override
    protected void onEnd() {}

    protected abstract Command getBlueprint();
}
