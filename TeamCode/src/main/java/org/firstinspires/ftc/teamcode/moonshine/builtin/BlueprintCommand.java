package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public abstract class BlueprintCommand extends SerialCommand {
    public BlueprintCommand() {
        super();
        children = new Command[]{ getBlueprint() };
    }

    abstract Command getBlueprint();
}
