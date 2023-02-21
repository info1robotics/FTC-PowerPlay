package org.firstinspires.ftc.teamcode.moonshine;

public class SharedSubsystem<T extends Subsystem> extends SharedVar<T> {
    public SharedSubsystem(Class<T> T) {
        super(T.getName());
    }
}
