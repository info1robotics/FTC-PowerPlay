package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Subsystem;

public class ContinuousReuseCommand extends ParallelCommand {
    public ContinuousReuseCommand(Subsystem... children) {
        super(Behavior.CONTINUOUS, children);
    }
}
