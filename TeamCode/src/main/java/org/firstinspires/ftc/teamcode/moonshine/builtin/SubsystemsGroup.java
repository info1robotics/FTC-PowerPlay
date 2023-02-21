package org.firstinspires.ftc.teamcode.moonshine.builtin;

import org.firstinspires.ftc.teamcode.moonshine.Subsystem;

public class SubsystemsGroup extends Subsystem {

    @Override
    protected void onStart() {
        stepThroughSubsystems();
    }

    @Override
    protected void onTick() {
        stepThroughSubsystems();
    }

    @Override
    protected void onEnd() {

    }

    @Override
    protected void onInitStart() {
        initStepThroughSubsystems();
    }

    @Override
    protected void onInitTick() {
        initStepThroughSubsystems();
    }

    @Override
    protected void onInitEnd() {

    }

    private void initStepThroughSubsystems() {
        for(Subsystem subsystem : childrenSubsystems) {
            subsystem.initStep();
        }
    }

    private void stepThroughSubsystems() {
        for(Subsystem subsystem : childrenSubsystems) {
            subsystem.step();
        }
    }
}
