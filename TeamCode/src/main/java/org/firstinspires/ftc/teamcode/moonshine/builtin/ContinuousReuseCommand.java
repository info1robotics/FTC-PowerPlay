package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Command;

public class ContinuousReuseCommand extends ParallelCommand {
    public ContinuousReuseCommand(Command... children) {
        super(Behavior.CONTINUOUS, children);
    }
}
